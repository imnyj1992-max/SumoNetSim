from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict

# 이 예제는 차량과 RSU가 주기적으로 HELLO 메시지를 교환하여 위치 정보를 수집하는 시뮬레이션을 구현합니다.

# 이 예제의 목적
# 1) 노드가 생성될 때 특정 작업을 수행하는 방법
# 2) 수신한 정보를 테이블에 저장하는 방법
# 3) 노드마다 주기적인 작업을 예약하는 방법
# 4) 노드 간 메시지 충돌 방지 방법
# 5) payload에 다양한 데이터를 담는 방법

######################## Start Function ########################
INTERVAL_TIME = 5.0  # 초

class PositionTable:
    def __init__(self) -> None:
        self.map: Dict[str, tuple[float, float]] = {}

######################### Implement Routing Protocol #########################
class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)
        self.position_table = PositionTable()        
        self._hello_running: bool = False

    # at created 메서드는 차량이 맵의 끝에서 처음 생성될 때 호출됩니다.
    def at_created(self) -> None:
        self._ensure_hello_loop()

    # hello 메시지가 동시에 전송되는 것을 방지하기 위해 차량마다 약간의 랜덤한 지터를 추가합니다.
    def _ensure_hello_loop(self) -> None:
        if self._hello_running or self.sim is None: return
        self._hello_running = True
        self._hello_jitter = random.uniform(0.0, 0.5)
        self.sim.schedule_event(self.sim.current_time + INTERVAL_TIME + self._hello_jitter, self._hello_tick)

    # 지터로 인해서 각 차량은 각기 다른 시간에 HELLO 메시지를 전송하게 됩니다.
    # 단, 각 차량은 일정한 주기로 HELLO 메시지를 전송합니다.
    # 예시1) 지터: 0.1234초, 첫 HELLO 메시지 전송 시간: 5.1234초, 두 번째 HELLO 메시지 전송 시간: 10.1234초
    # 예시2) 지터: 0.4321초, 첫 HELLO 메시지 전송 시간: 5.4321초, 두 번째 HELLO 메시지 전송 시간: 10.4321초
    def _hello_tick(self) -> None:
        if not self._hello_running: return        
        self.broadcast(net.PacketType.HELLO, payload={"id": self.id, "pos": self.pos}, size_bytes=64)        
        self.sim.schedule_event(self.sim.current_time + INTERVAL_TIME, self._hello_tick)

    # Hello 메시지를 받은 차량은 position table을 업데이트합니다.
    def handle_hello(self, pkt: net.Packet) -> None:
        self.position_table.map[pkt.src.id] = tuple(pkt.payload["pos"])
        print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} received HELLO from {pkt.src.id}")

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.position_table = PositionTable()

    # Hello 메시지를 받은 RSU는 position table을 업데이트합니다.
    def handle_hello(self, pkt: net.Packet) -> None:
        self.position_table.map[pkt.src.id] = tuple(pkt.payload["pos"])
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} received HELLO from {pkt.src.id}")

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=None)
    netsim.run()