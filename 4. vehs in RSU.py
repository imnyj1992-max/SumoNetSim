from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List, Tuple

# 이 예제는 테이블을 사용하지 않고 RSU 범위 내의 차량을 확인하는 기능을 시연합니다.
# 올바른 접근 방법은 아니지만, 필요한 경우에 사용할 수 있도록 예제를 제공합니다.

# 이 예제의 목적
# 1) REQUEST 메시지를 주기적으로 전달하는 방법
# 2) REQUEST 메시지가 전달되었을 때, 반복을 멈추는 방법
# 3) 차량이 자신의 모드를 변경하는 방법
# 4) 전역 변수를 통하여, RSU가 자신의 범위 내에 있는 차량 목록을 확인하는 방법

######################## Config ########################
START_TIME = 100.0
REQ_PERIOD = 5.0
REQ_SIZE = 64
REPLY_SIZE = 32
RSU_RANGE = 800.0

# 시작 시, 100초 후에 랜덤한 차량이 주변에 REQUEST 메시지를 브로드캐스트합니다.
def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    def kick_off():
        if not vehicles: return
        veh = random.choice(list(vehicles.values()))
        
        print(f"[{sim.current_time:.6f}s] Select target vehicle: {veh.id}. Start periodic REQUEST.")

        veh.broadcast(net.PacketType.REQUEST, payload={"veh_id": veh.id, "pos": veh.pos}, size_bytes=REQ_SIZE)
        veh.start_req()

    sim.schedule_event(START_TIME, kick_off)

######################### Implement Routing Protocol #########################
class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)
        self._req_active: bool = False
        self.mode = None

    # 이후, 자신의 모드를 Request로 바꾸고, 주기적으로 REQUEST 메시지를 브로드캐스트합니다.
    def start_req(self) -> None:
        self.mode = "REQUEST"
        self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._req_tick)

    def _req_tick(self) -> None:
        if not self.mode == "REQUEST": return
        self.broadcast(net.PacketType.REQUEST, payload={"veh_id": self.id, "pos": self.pos}, size_bytes=REQ_SIZE)
        print(f"[{self.sim.current_time:.6f}s] target vehicle: {self.id} REQUEST.")
        if self.mode == "REQUEST": self.sim.schedule_event(self.sim.current_time + REQ_PERIOD, self._req_tick)

    # 해당 차량이 RSU로부터 ACK 메시지를 받으면, 자신의 모드를 Download로 바꾸고, REQUEST 메시지 브로드캐스트를 중단합니다.
    def handle_ack(self, pkt: net.Packet) -> None:
        self.mode = "DOWNLOAD"
        print(f"[{self.sim.current_time:.6f}s] Vehicle {self.id} received REPLY from {pkt.src.id} -> stop REQUEST")

    # 차량은 handle_request()가 구현되지 않아서 차량 간에 전송되는 REQUEST 메시지를 무시합니다.

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True

    # RSU는 차량으로부터 REQUEST 메시지를 받으면, ACK 메시지로 응답하고, 자신의 범위 내에 있는 차량 목록을 출력합니다.
    def handle_request(self, pkt: net.Packet) -> None:
        print(f"[{self.sim.current_time:.6f}s] RSU {self.id} received REQUEST from {pkt.src.id}")
        self.send_packet(net.Packet(pkt_type=net.PacketType.ACK, src=self, dst=pkt.src, payload={"reply": True}, size_bytes=REPLY_SIZE))
        v_list = self.GetVehiclesInRange()
        print(len(v_list), "Vehicles :", v_list)

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()