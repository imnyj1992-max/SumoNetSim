from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List, Tuple

# 이 예제는 V2I Precaching 논문 아이디어를 단순화하여 구현한 것입니다.
# 차량의 요청을 받은 RSU는 30% 확률로 미리 캐싱된 콘텐츠를 제공하고, 그렇지 않은 경우에는 서버에서 콘텐츠를 가져와서 차량에 전달합니다.
# 다른 점은 요청 차량의 next RSU를 받아서 해당 RSU가 요청된 Content를 캐싱하도록 합니다.
# 여러 대의 상황에서 구현을 원할 경우, REQUEST를 at_created()를 통해서 구현하면 됩니다.

# 이 예제의 목적
# 1) RSU의 Precaching 동작 이해
# 2) Precaching을 위한 차량의 추가 정보 요구 확인
# 3) 패킷 타입 외에 Payload를 통한 추가 정보 전달 방법 이해
# 4) 차량의 Next RSU 정보 받아오는 메서드 사용

# 차량이 사용할 수 있는 메서드
# - GetSpeed(v_id) -> float
# - GetAcceleration(v_id) -> float
# - GetPosition(v_id) -> Tuple[float, float] 타입의 x, y 좌표 반환
# - GetRoutes(v_id) -> List[str] 타입의 RSU ID 리스트 반환
# - GetNextRSU(v_id) -> str 타입의 RSU ID 반환, 없으면 None 반환

######################## Config ########################
START_TIME    = 100.0
REQ_PERIOD    = 1.0
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
        if self.b_log: print(f"[{self.sim.current_time:.6f}s] Vehicle({self.id}) REQUEST.")
        self.finding_rsu(net.PacketType.REQUEST, payload={"veh_id": self.id, "c_id_req": CONTENT_ID, "pos": self.pos, "next_rsu": net.GetNextRSU(self.id)}, size_bytes=REQ_SIZE)
        self._dwell_start = self.sim.current_time
        if self.mode == "REQUEST":
            self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._request_tick)

    def handle_ack(self, pkt: net.Packet) -> None:
        self.mode = "DOWNLOAD"
        print(f"[{self.sim.current_time:.6f}s] REPLY [R{pkt.src.id} → V{self.id}] to stop REQUEST")

    def handle_data(self, pkt: net.Packet) -> None:
        self.bytes_received += pkt.size_bytes
        self.bytes_remained = CONTENT_SIZE - self.bytes_received

        mb = self.bytes_received / (1024 * 1024)
        dwell = self.sim.current_time - self._dwell_start
        self.mode = None
        if self.bytes_received >= CONTENT_SIZE:
            print(f"[{self.sim.current_time:.6f}s] DATA [R{pkt.src.id} → V{self.id}] {CONTENT_ID} Received {mb:.2f} MB from  (Complete, Dwell {dwell:.3f}s)")
        else:
            print(f"[{self.sim.current_time:.6f}s] DATA [R{pkt.src.id} → V{self.id}] {CONTENT_ID} Received {mb:.2f} MB from  (Progress, Dwell {dwell:.3f}s)")
            self.start_request()

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True
        self.cache: Dict[int, int] = {}
        # RSU 중 랜덤하게 5% 확률로 요청될 content를 캐시에 보유 : 더 적은 확률로 캐시를 보유하도록 설정 for Precaching 효과 극대화
        if random.random() < 0.05: self.cache[CONTENT_ID] = CONTENT_SIZE 
        self._pending_vehicle = None

    # 차량으로부터 REQUEST 메시지를 받는 경우
    def handle_request(self, pkt: net.Packet) -> None:
        src = pkt.src
        print(f"[{self.sim.current_time:.6f}s] REQUEST [V{src.id} → R{self.id}] c_id={CONTENT_ID}")

        # 1) ACK 전송 : 차량의 REQUEST 반복을 멈추기 위해
        ack = net.Packet(net.PacketType.ACK, self, src, payload={"reply": True}, size_bytes=REPLY_SIZE)
        self.send_packet(ack)

        # 2) 로컬 캐시 제공 : RSU가 캐시를 가지고 있으면 바로 전송
        if CONTENT_ID in self.cache:
            remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
            self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=src, payload={"c_id": CONTENT_ID, "from": f"RSU:{self.id} (cached)"}, size_bytes=remaining))
            return

        # 3) 서버에서 가져오기 : RSU가 캐시를 가지고 있지 않으면 서버에서 가져와서 차량에 전달하기 위해 서버로 요청
        server = next((n for n in self.sim.nodes if getattr(n, "is_server", False)), None)
        self._pending_vehicle = src
        remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
        self.send_packet(net.Packet(pkt_type=net.PacketType.REQUEST, src=self, dst=server, payload={"c_id_req": CONTENT_ID, "c_size": remaining, "veh_id": src.id, "from_rsu": self.id}, size_bytes=REQ_SIZE))

        # 4) 요청 차량의 next RSU가 있으면, 해당 RSU가 content를 캐싱하도록 설정
        next_rsu_id = pkt.payload.get("next_rsu")
        if next_rsu_id is None: return
        next_rsu = net.rsu_dict.get(next_rsu_id)
        self.send_packet(net.Packet(pkt_type=net.PacketType.PRECACHE, src=self, dst=next_rsu, payload={"c_id": CONTENT_ID, "c_size": remaining, "type": "LET"}, size_bytes=REQ_SIZE))

    # 서버로부터 Data 메시지를 받는 경우
    def handle_data(self, pkt: net.Packet) -> None:
        if self._pending_vehicle is None: return
        print(f"[{self.sim.current_time:.6f}s] DATA [Server → R{self.id}] c_id={CONTENT_ID} from {pkt.src.id}")
        # pending vehicle를 기반으로 요청 차량에게 데이터 전송
        v = self._pending_vehicle
        mb = pkt.size_bytes / (1024 * 1024)
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=v, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=pkt.size_bytes,))
        self._pending_vehicle = None

    def handle_precache(self, pkt: net.Packet) -> None:
        # 직전 RSU로부터 LET 타입의 PRECACHE 메시지를 받는 경우, Precaching 시작
        if pkt.payload.get("type") == "LET":
            print(f"[{self.sim.current_time:.6f}s] PRECACHE [R{pkt.src.id} → R{self.id}] LET type")
            # 1) 로컬 캐시 제공 : RSU가 캐시를 가지고 있으면 바로 전송
            if CONTENT_ID in self.cache: return

            # 2) 서버에서 가져오기 : RSU가 캐시를 가지고 있지 않으면 서버에서 가져와서 Precache하기 위해 서버로 요청
            server = next((n for n in self.sim.nodes if getattr(n, "is_server", False)), None)
            remaining = pkt.payload.get("c_size", CONTENT_SIZE)
            self.send_packet(net.Packet(pkt_type=net.PacketType.PRECACHE, src=self, dst=server, payload={"c_id": CONTENT_ID, "c_size": remaining, "type": "REQUEST"}, size_bytes=REQ_SIZE))
        # 3) 서버로부터 DATA 타입의 PRECACHE 메시지를 받는 경우, 캐시에 저장            
        elif pkt.payload.get("type") == "DATA":
            print(f"[{self.sim.current_time:.6f}s] PRECACHE [Server → R{self.id}] DATA type")
            self.cache[CONTENT_ID] = CONTENT_SIZE

class ContentServer(net.Node):
    def __init__(self, node_id: str="SERVER", pos: Tuple[float, float]=(1e6, 1e6)) -> None:
        super().__init__(node_id, pos=pos, comm_range=0.0)
        # Content Server 노드 표시용 플래그
        self.is_server = True

    # RSU로부터 REQUEST 메시지를 받는 경우, 즉시 Data 메시지로 응답
    def handle_request(self, pkt: net.Packet) -> None:
        c_size = pkt.payload.get("c_size")
        print(f"[{self.sim.current_time:.6f}s] REQUEST [R{pkt.src.id} → Server] c_id={CONTENT_ID}")
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=pkt.src, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=c_size))

    # RSU로부터 REQUEST 타입의 PRECACHE 메시지를 받는 경우, 즉시 Data 메시지로 응답
    def handle_precache(self, pkt: net.Packet) -> None:
        if pkt.payload.get("type") == "REQUEST":
            print(f"[{self.sim.current_time:.6f}s] PRECACHE [R{pkt.src.id} → Server] REQUEST type")
            c_size = pkt.payload.get("c_size")
            self.send_packet(net.Packet(pkt_type=net.PacketType.PRECACHE, src=self, dst=pkt.src, payload={"c_id": CONTENT_ID, "type": "DATA"}, size_bytes=c_size))

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()