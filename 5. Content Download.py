from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List, Tuple

# 이 예제는 기본적인 Content Download 시나리오를 구현합니다.

# 이 예제의 목적
# 1) 차량이 Content를 다 받지 못할 경우, 다시 REQUEST를 시작하는 방법
# 2) RSU가 캐시를 가지고 있을 때와 없을 때의 동작 차이
# 3) RSU가 서버로부터 content를 가져와서 차량에 전달하는 방법
# 4) Content Server 노드를 추가하는 방법
# 5) Content Server 식별 방법

######################## Config ########################
START_TIME    = 100.0
REQ_PERIOD    = 1.0        # 차량의 REQUEST 주기
REQ_SIZE      = 64
REPLY_SIZE    = 32
CONTENT_ID    = 42
CONTENT_SIZE  = 1024 * 1024 * 1024  # 1 GB

######################## Start Hook ########################
# 시작 시, 콘텐츠 서버 노드 추가 및 랜덤한 차량 선택 후 REQUEST 시작
def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    # 콘텐츠 서버 노드 추가!!
    sim.add_node(ContentServer())

    def kick_off():
        if not vehicles: return
        veh = random.choice(list(vehicles.values()))
        print(f"[{sim.current_time:.6f}s] Select target vehicle: {veh.id}. Start periodic REQUEST.")
        veh.mode = None
        veh.start_request()
    sim.schedule_event(START_TIME, kick_off)

######################## Nodes #########################
class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)
        self._req_active = False
        self.mode = None
        self.bytes_remained = CONTENT_SIZE  # 받아야 하는 바이트 수
        self.bytes_received = 0             # 지금까지 받은 바이트 수
        self._dwell_start = None            # 차량의 dwell time 측정용
        self.b_log = False                  # 로그 출력 여부 (True/False) -> 디버깅용

    def start_request(self) -> None:
        self.mode = "REQUEST"
        self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._request_tick)

    def _request_tick(self) -> None:
        if not self.mode == "REQUEST": return
        if self.b_log: print(f"[{self.sim.current_time:.6f}s] target vehicle: {self.id} REQUEST.")
        # 그냥 Broadcast 보다는 RSU를 탐색하는 메서드 사용 가능
        self.finding_rsu(net.PacketType.REQUEST, payload={"veh_id": self.id, "c_id_req": CONTENT_ID, "pos": self.pos}, size_bytes=REQ_SIZE)
        # dwell time 초기화
        self._dwell_start = self.sim.current_time
        # REQUEST 모드일 때만 주기적 REQUEST 재스케줄
        if self.mode == "REQUEST":
            self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._request_tick)

    # RSU를 찾아서 ACK를 받으면, 모드를 DOWNLOAD로 변경
    def handle_ack(self, pkt: net.Packet) -> None:
        self.mode = "DOWNLOAD"
        print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} received REPLY from {pkt.src.id} -> stop REQUEST")

    # RSU로부터 Data 메시지를 받는 경우
    # Data의 total size를 다 받거나, 통신 범위를 벗어나면 수신 함수가 실행됨
    def handle_data(self, pkt: net.Packet) -> None:
        self.bytes_received += pkt.size_bytes
        self.bytes_remained = CONTENT_SIZE - self.bytes_received

        mb = self.bytes_received / (1024 * 1024)
        dwell = self.sim.current_time - self._dwell_start
        self.mode = None
        # 다 받은 경우에는 Complete 출력
        if self.bytes_received >= CONTENT_SIZE:
            print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} Data {CONTENT_ID} Received {mb:.2f} MB from {pkt.src.id} (Complete, Dwell {dwell:.3f}s)")
        # 다 받지 못한 경우에는 Progress 출력 후, 다시 REQUEST 시작
        else:
            print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} Data {CONTENT_ID} Received {mb:.2f} MB from {pkt.src.id} (Progress, Dwell {dwell:.3f}s)")
            self.start_request()

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True
        self.cache: Dict[int, int] = {}
        if random.random() < 0.3: self.cache[CONTENT_ID] = CONTENT_SIZE # RSU 중 랜덤하게 30% 확률로 요청될 content를 캐시에 보유
        self._pending_vehicle = None

    # 차량으로부터 REQUEST 메시지를 받는 경우
    def handle_request(self, pkt: net.Packet) -> None:
        src = pkt.src
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} received REQUEST c_id={CONTENT_ID} from {src.id}")

        # 1) ACK 전송 : 차량의 REQUEST 반복을 멈추기 위해
        ack = net.Packet(net.PacketType.ACK, self, src, payload={"reply": True}, size_bytes=REPLY_SIZE)
        self.send_packet(ack)

        # 2) 로컬 캐시 제공 : RSU가 캐시를 가지고 있으면 바로 전송
        if CONTENT_ID in self.cache:
            print(f"[{self.sim.current_time:.6f}s] RSU {self.id} serves c_id={CONTENT_ID} from LOCAL CACHE -> {src.id}")
            remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
            self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=src, payload={"c_id": CONTENT_ID, "from": f"RSU:{self.id} (cached)"}, size_bytes=remaining))
            return

        # 3) 서버에서 가져오기 : RSU가 캐시를 가지고 있지 않으면 서버에서 가져와서 차량에 전달하기 위해 서버로 요청
        server = next((n for n in self.sim.nodes if getattr(n, "is_server", False)), None)
        # pending vehicle에 요청 차량 정보 저장
        self._pending_vehicle = src
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} fetch c_id={CONTENT_ID} from SERVER (fiber 13-hop)")
        remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
        self.send_packet(net.Packet(pkt_type=net.PacketType.REQUEST, src=self, dst=server, payload={"c_id_req": CONTENT_ID, "c_size": remaining, "veh_id": src.id, "from_rsu": self.id}, size_bytes=REQ_SIZE))

    # 서버로부터 Data 메시지를 받는 경우
    def handle_data(self, pkt: net.Packet) -> None:
        if self._pending_vehicle is None: return
        # pending vehicle를 기반으로 요청 차량에게 데이터 전송
        v = self._pending_vehicle
        mb = pkt.size_bytes / (1024 * 1024)
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} cached c_id={CONTENT_ID} {mb:.2f} MB from {pkt.src.id}")
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=v, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=pkt.size_bytes,))
        self._pending_vehicle = None

class ContentServer(net.Node):
    def __init__(self, node_id: str="SERVER", pos: Tuple[float, float]=(1e6, 1e6)) -> None:
        super().__init__(node_id, pos=pos, comm_range=0.0)
        # Content Server 노드 표시용 플래그
        self.is_server = True

    # RSU로부터 REQUEST 메시지를 받는 경우, 즉시 Data 메시지로 응답
    def handle_request(self, pkt: net.Packet) -> None:
        c_size = pkt.payload.get("c_size")
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=pkt.src, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=c_size))

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()