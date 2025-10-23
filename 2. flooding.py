from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List

# 이 예제는 단순 flooding 예제입니다.
# Hello 메시지를 받은 노드가 똑같이 주변에 Hello 메시지를 뿌려주는 방식입니다.

# 이 예제의 목적
# 1) 메시지를 수신한 뒤 action으로 또 다른 메시지를 보내는 방법

######################## Start Function ########################
# 시작 시, 100초 후에 랜덤한 차량이 주변에 HELLO 메시지를 브로드캐스트합니다.
# Hello[src → all]
START_TIME = 100.0

def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    def send_requests() -> None:
        if not vehicles: return
        src = random.choice(list(vehicles.values()))
        src.broadcast(net.PacketType.HELLO, payload={}, size_bytes=128)

        print(f"[{sim.current_time:.6f}s] HELLO [{src.id} broadcast]")

    sim.schedule_event(START_TIME, send_requests)

######################### Implement Routing Protocol #########################
class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)

    # Hello 메시지를 받은 차량은 다시 주변에 Hello 메시지를 브로드캐스트합니다.
    # Hello[src → all]
    def handle_hello(self, pkt: net.Packet) -> None:
        print(f"[{self.sim.current_time:.6f}s] HELLO [{pkt.src.id} → {self.id}]")        
        self.broadcast(net.PacketType.HELLO, payload={}, size_bytes=128)


class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
    
    # Hello 메시지를 받은 RSU는 다시 주변에 Hello 메시지를 브로드캐스트합니다.
    # Hello[src → all]
    def handle_hello(self, pkt: net.Packet) -> None:
        print(f"[{self.sim.current_time:.6f}s] HELLO [{pkt.src.id} → {self.id}]")
        self.broadcast(net.PacketType.HELLO, payload={}, size_bytes=128)


###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()