from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List, Tuple

# 이 예제는 여러 대의 차량이 Content를 Download 하는 시나리오를 구현합니다.

# 이 예제의 목적
# 1) 여러 대의 차량이 Content를 요청하는 시나리오에 대한 이해

######################## Config ########################
REQ_PERIOD    = 500.0
REQ_SIZE      = 64
REPLY_SIZE    = 32
CONTENT_ID    = 42
CONTENT_SIZE  = 1024 * 1024 * 1024  # 1 GB

######################## Start Hook ########################
# 시작 시, 콘텐츠 서버 노드 추가
def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    sim.add_node(ContentServer())

######################## Nodes #########################
class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)
        self._req_active = False
        self.mode = None
        self.bytes_remained = CONTENT_SIZE
        self.bytes_received = 0
        self._dwell_start = None
        self.b_log = False
        self.t_req = 0.0

    # 노드가 맵 끝에서 생성되면서 부터 요청 스케줄링
    def at_created(self) -> None:
        self.t_req = self.sim.current_time + random.uniform(0, REQ_PERIOD)
        self.sim.schedule_event(self.t_req, self.start_request)

    # 너무 많은 차량이 REQUEST 메시지를 보내지 않도록 제어
    def start_request(self) -> None:
        if random.random() < 0.1:
            self.mode = "REQUEST"
            self._request_tick()
        else:
            self.t_req = self.sim.current_time + random.uniform(0, REQ_PERIOD)
            self.sim.schedule_event(self.t_req, self.start_request)

    def _request_tick(self) -> None:
        if not self.mode == "REQUEST": return
        if self.b_log: print(f"[{self.sim.current_time:.6f}s] target vehicle: {self.id} REQUEST.")
        self.finding_rsu(net.PacketType.REQUEST, payload={"veh_id": self.id, "c_id_req": CONTENT_ID, "remaining":self.bytes_remained}, size_bytes=REQ_SIZE)
        self._dwell_start = self.sim.current_time

        if self.mode == "REQUEST":
            self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._request_tick)

    def handle_ack(self, pkt: net.Packet) -> None:
        self.mode = "DOWNLOAD"
        if self.b_log: print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} received REPLY from {pkt.src.id} -> stop REQUEST")

    # Data 메시지를 받고 Content 전송 완료 여부 확인
    def handle_data(self, pkt: net.Packet) -> None:
        self.bytes_received += pkt.size_bytes
        self.bytes_remained = CONTENT_SIZE - self.bytes_received

        mb = self.bytes_received / (1024 * 1024)
        dwell = (self.sim.current_time - self._dwell_start) if self._dwell_start is not None else 0.0
        self.mode = None
        fr = pkt.payload.get("from", "Unknown")
        if self.bytes_received >= CONTENT_SIZE:
            print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} Data {CONTENT_ID} Received {mb:.2f} MB from {fr} (Complete, Dwell {dwell:.3f}s)")
        else:
            print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} Data {CONTENT_ID} Received {mb:.2f} MB from {fr} (Progress, Dwell {dwell:.3f}s)")
            self.start_request()

# RSU는 거의 비슷한 동작을 함
class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True
        self.cache: Dict[int, int] = {}
        if random.random() < 0.3: self.cache[CONTENT_ID] = CONTENT_SIZE
        self.pending_table: dict[net.Node, tuple[str, int]] = {}

    def handle_request(self, pkt: net.Packet) -> None:
        src = pkt.src
        veh_id = src.id
        need = pkt.payload.get("remaining", CONTENT_SIZE)
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} REQ from {veh_id} need={need/(1024*1024):.1f}MB")

        self.send_packet(net.Packet(net.PacketType.ACK, self, src, payload={"reply": True}, size_bytes=REPLY_SIZE))
            
        if CONTENT_ID in self.cache:
            self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=src, payload={"c_id": CONTENT_ID, "from": self.id}, size_bytes=need,))
            return

        # 다만, pending vehicle이 여러대 일 수 있기 때문에, table로 관리
        self.pending_table[src] = (veh_id, need)
        server = next((n for n in self.sim.nodes if getattr(n, "is_server", False)), None)
        self.send_packet(net.Packet(pkt_type=net.PacketType.REQUEST, src=self, dst=server, payload={"c_id_req": CONTENT_ID, "c_size": CONTENT_SIZE, "veh_id": src.id, "from_rsu": self.id}, size_bytes=REQ_SIZE))

    # Content Server로부터 DATA 메시지를 받으면, pending table에 있는 차량들에게 전송
    def handle_data(self, pkt: net.Packet) -> None:
        mb = pkt.size_bytes / (1024 * 1024)
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} cached c_id={CONTENT_ID} {mb:.2f} MB from {pkt.src.id}")
        self.cache[CONTENT_ID] = CONTENT_SIZE
        for veh, (id, need) in self.pending_table.items():
            if veh not in self.sim.nodes: continue
            self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=veh, payload={"c_id": CONTENT_ID, "from": "Server"}, size_bytes=need,))
        self._pending_vehicle = None

# Content Server의 동작은 이전과 같음
class ContentServer(net.Node):
    def __init__(self, node_id: str="SERVER", pos: Tuple[float, float]=(1e6, 1e6)) -> None:
        super().__init__(node_id, pos=pos, comm_range=0.0)
        self.is_server = True

    def handle_request(self, pkt: net.Packet) -> None:
        c_size = pkt.payload.get("c_size")
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=pkt.src, payload={"c_id": CONTENT_ID, "from": "Server"}, size_bytes=c_size))

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()