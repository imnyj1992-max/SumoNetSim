from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List

# 이 예제는 매우 간단한 HELLO 메시지 교환 시나리오를 구현합니다.

# 이 예제의 목적
# 1) 시작과 동시에 시행되는 함수의 이해 + 랜덤한 차량 선택
# 2) 차량과 RSU 노드에서의 별도 메서드 구현
# 3) 브로드캐스트 메시지 전송 및 수신 처리

######################## Start Function ########################

# 시작 시, 100초 후에 랜덤한 차량이 주변에 HELLO 메시지를 브로드캐스트합니다.
# HELLO[src → all]

START_TIME = 100.0

def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    def send_requests() -> None:
        if not vehicles: return
        src = random.choice(list(vehicles.values()))
        src.broadcast(net.PacketType.HELLO, payload={}, size_bytes=128)

        print(f"[{sim.current_time:.6f}s] HELLO [{src.id} bradcast]")

    sim.schedule_event(START_TIME, send_requests)

######################### Implement Routing Protocol #########################

class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)

    # 차량이 HELLO 메시지를 수신하면, 메시지의 출발지와 자신의 ID를 출력합니다.    
    def handle_hello(self, pkt: net.Packet) -> None:
        print(f"[{self.sim.current_time:.6f}s] HELLO [{pkt.src.id} → {self.id}]")


class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
    
    # RSU가 HELLO 메시지를 수신하면, 메시지의 출발지와 자신의 ID를 출력합니다.
    def handle_hello(self, pkt: net.Packet) -> None:
        print(f"[{self.sim.current_time:.6f}s] HELLO [{pkt.src.id} → {self.id}]")

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()