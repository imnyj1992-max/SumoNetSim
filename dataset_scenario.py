from __future__ import annotations
import os, csv, time, math, random
from typing import Dict, List, Tuple, Any, Optional
import src.NetSim as net
os.environ.setdefault("SUMO_USE_LIBSUMO", "1")
import libsumo as sumo

BUFFER_SIZE = 50
T_INIT = 0.0
PER_REQ = 300
netsim = None

def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    global T_INIT
    T_INIT = t_init
    def print_start():
        print(f"Start to record {sim.current_time} s")
    sim.schedule_event(t_init, print_start)


class VehicleNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=200.0)
        self.cur_rsu: Optional[net.Node] = None
        self.prev_rsu: Optional[net.Node] = None
        self.entry_time: Optional[float] = None
        self.dwell_time: Optional[float] = None
        self.next_entry_time: Optional[float] = None
        self.exit_time: Optional[float] = None
        self.req_time: Optional[float] = None
        self.report_sent: bool = False
        self.mode = None
        self.b_log = True

    def at_created(self) -> None:
        def enter_request_mode():
            self.mode = "REQUEST"
            self.sim.schedule_event(self.sim.current_time + random.uniform(0, PER_REQ), self.send_request)

        if self.sim.current_time < T_INIT:
            self.sim.schedule_event(T_INIT, enter_request_mode)
        else:
            enter_request_mode()

    def send_request(self) -> None:
        if self.mode == "REQUEST":
            self.finding_rsu(net.PacketType.REQUEST, payload={'veh_id': self.id}, size_bytes=32)
            self.entry_time = self.sim.current_time
            self.sim.schedule_event(self.sim.current_time + 1, self.send_request)

    def handle_ack(self, pkt: net.Packet) -> None:
        self.mode = "DOWNLOAD"
        rsu = pkt.src
        if self.cur_rsu is None and self.prev_rsu is None:
            if self.b_log: print(f"A {self.id} < {rsu.id:<5}\t{self.sim.current_time}\t+++++")
            self.cur_rsu = rsu
            self.req_time = self.sim.current_time
            self.sim.schedule_event(self.sim.current_time + self.sim.step, self.check_range)
            return
        if self.cur_rsu is None and self.prev_rsu is not None:
            if rsu == self.prev_rsu:
                if self.b_log: print(f"R {self.id} < {rsu.id:<5}\t{self.sim.current_time}\t--REENTRY--")
                self.cur_rsu = rsu
                self.prev_rsu = None
                self.exit_time = None
                self.next_entry_time = None
                self.report_sent = False
                self.sim.schedule_event(self.sim.current_time + self.sim.step, self.check_range)
                return
            if self.b_log: print(f"C {self.id} < {rsu.id:<5}\t{self.sim.current_time}\t+++++++++++++++")
            self.cur_rsu = rsu
            self.next_entry_time = self.sim.current_time - self.req_time
            if self.exit_time is not None and self.b_log:
                print(f"TRANSITION {self.id} < {self.prev_rsu.id}->{rsu.id} dt = {self.next_entry_time:.2f}")
            self.report_sent = False
            self.sim.schedule_event(self.sim.current_time + self.sim.step, self.check_range)
            return
        self.cur_rsu = rsu
        self.sim.schedule_event(self.sim.current_time + self.sim.step, self.check_range)

    def check_range(self) -> None:
        if self.cur_rsu is None:
            return
        dist = self.distance_to(self.cur_rsu)
        if dist > self.cur_rsu.comm_range:
            self.mode = "REQUEST"
            if self.prev_rsu is None:
                if self.b_log: print(f"B {self.id} from {self.cur_rsu.id}\t{self.sim.current_time}\t++++++++++")
                self.dwell_time = self.sim.current_time - self.req_time
                self.exit_time = self.sim.current_time
                self.prev_rsu = self.cur_rsu
                self.cur_rsu = None
                self.sim.schedule_event(self.sim.current_time + 1, self.send_request)
                return
            if not self.report_sent:
                if self.b_log: print(f"D {self.id} from {self.cur_rsu.id}\t{self.sim.current_time}\t++++++++++++++++++++")
                exit_time = self.sim.current_time - self.req_time
                payload = {'veh_id': self.id, 'prev_rsu': self.prev_rsu.id, 'dwell_time': self.dwell_time, 'next_entry_time': self.next_entry_time, 'exit_time': exit_time}
                self.send_direct(net.Packet(pkt_type=net.PacketType.REPORT, src=self, dst=self.cur_rsu, payload=payload, size_bytes=128))
                self.report_sent = True
                self.prev_rsu = None
                self.next_entry_time = None
            self.dwell_time = None
            self.cur_rsu = None
            self.sim.schedule_event(self.sim.current_time + 1, self.send_request)
            return
        self.sim.schedule_event(self.sim.current_time + 1, self.check_range)

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float] = (0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True
        self.pending_records: Dict[str, Dict[str, Any]] = {}
        self.buffer: List[Dict[str, Any]] = []
        self.buffer_size = BUFFER_SIZE
        self._dwell_map: Dict[str, float] = {}
        self.b_log = True

    def handle_request(self, pkt: net.Packet) -> None:
        veh_node = pkt.src
        veh_id = veh_node.id
        route_rsus = net.GetRoutes(veh_id)
        if self.id in route_rsus:
            idx = route_rsus.index(self.id)
        else:
            idx = 0
        remaining = [r for r in route_rsus[idx + 1:] if r in net.rsu_dict]
        if len(remaining) < 2:
            return
        features = self._compute_features(veh_node)
        next_rsu = net.GetNextRSU(veh_id)
        record = {
            'veh_id': veh_id,
            'cur_rsu': self.id,
            'next_rsu': next_rsu,
            'features': features,
            'targets': {}
        }
        self.pending_records[veh_id] = record
        self.send_direct(net.Packet(pkt_type=net.PacketType.ACK, src=self, dst=veh_node, payload={'reply': True}, size_bytes=32))

    def handle_report(self, pkt: net.Packet) -> None:
        data = pkt.payload
        prev_rsu_id = data.get('prev_rsu')
        veh_id = data.get('veh_id')
        if not veh_id:
            return
        if self.b_log: print(f"REPORT   {self.id} < {veh_id}\t{self.sim.current_time}")
        if prev_rsu_id != self.id:
            prev_node = net.rsu_dict.get(prev_rsu_id)
            if prev_node:
                self.send_packet(net.Packet(pkt_type=net.PacketType.REPORT, src=self, dst=prev_node, payload=data, size_bytes=128))
            return
        record = self.pending_records.pop(veh_id, None)
        if not record:
            return
        record['targets'] = {
            'dwell_time': data.get('dwell_time'),
            'next_entry_time': data.get('next_entry_time'),
            'exit_time': data.get('exit_time')
        }
        self.buffer.append(record)
        if self.b_log: print(f"{self.id}+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        if len(self.buffer) >= self.buffer_size:
            self._flush_buffer()

    def _flush_buffer(self) -> None:
        if not self.buffer:
            return
        fname = f"rsu_{self.id}.csv"
        file_path = os.path.join(os.getcwd(), "data", fname)
        file_exists = os.path.exists(file_path)
        with open(file_path, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            if not file_exists:
                header = ['veh_id', 'cur_rsu', 'next_rsu']
                feat_names = list(self.buffer[0]['features'].keys())
                header.extend(feat_names)
                header.extend(['dwell_time', 'next_entry_time', 'exit_time'])
                writer.writerow(header)
            for rec in self.buffer:
                row = [rec['veh_id'], rec['cur_rsu'], rec['next_rsu']]
                for k in rec['features']:
                    row.append(rec['features'][k])
                row.append(rec['targets'].get('dwell_time'))
                row.append(rec['targets'].get('next_entry_time'))
                row.append(rec['targets'].get('exit_time'))
                writer.writerow(row)
        self.buffer = []

    def _compute_features(self, veh_node: net.Node) -> Dict[str, Any]:
        features: Dict[str, Any] = {}
        features['r_cov'] = self.comm_range
        next_rsu_id: Optional[str] = net.GetNextRSU(veh_node.id)
        next_rsu = net.rsu_dict.get(next_rsu_id) if next_rsu_id else None
        dir_flag = 0
        try:
            lane_id = sumo.vehicle.getLaneID(veh_node.id)
            edge_id = sumo.lane.getEdgeID(lane_id)
            to_node_id = None
            try:
                edge_obj = net._network_cache.getEdge(edge_id) if net._network_cache else None
                if edge_obj is not None:
                    to_node_id = edge_obj.getToNode().getID()
            except Exception:
                to_node_id = None
            if next_rsu and to_node_id == next_rsu_id:
                dir_flag = -1
            else:
                dir_flag = 1
        except Exception:
            dir_flag = 0
        features['dirct'] = dir_flag
        # Distance to the boundary of the next RSU's communication range
        if next_rsu:
            dist_v_to_next = math.hypot(veh_node.pos[0] - next_rsu.pos[0], veh_node.pos[1] - next_rsu.pos[1])
            dist_v_to_cur = math.hypot(veh_node.pos[0] - self.pos[0], veh_node.pos[1] - self.pos[1])
            dist_cur_to_next = math.hypot(self.pos[0] - next_rsu.pos[0], self.pos[1] - next_rsu.pos[1])
            if dir_flag == -1:
                d_n_c = max(dist_v_to_next - next_rsu.comm_range, 0.0)
            else:
                d_n_c = max(dist_v_to_cur + dist_cur_to_next - next_rsu.comm_range, 0.0)
            features['d_n_c'] = d_n_c
        else:
            features['d_n_c'] = float('inf')
        # Count vehicles at this RSU heading to the same next RSU
        features['n_t_0'] = self._count_vehicles_to_next(next_rsu_id)
        # Counts for the three nearest RSUs excluding current and next
        if next_rsu is not None and hasattr(next_rsu, "rsu_row") and hasattr(next_rsu, "rsu_col"):
            base_r = next_rsu.rsu_row
            base_c = next_rsu.rsu_col
        else:
            base_r = None
            base_c = None
        grid = getattr(netsim, "rsu_grid", None)
        neighbors = [(-1, 0), (0, -1), (1, 0), (0, 1)]  # N, W, S, E 예시
        for idx, (dr, dc) in enumerate(neighbors):
            key = f"n_t_{idx}"   # n_t_0, n_t_1, n_t_2, n_t_3
            if base_r is None or base_c is None or grid is None:
                features[key] = 0
                continue
            nb = grid.get((base_r + dr, base_c + dc))
            if nb is None:
                features[key] = 0
            else:
                features[key] = self._count_vehicles_in_range_to_next(nb, next_rsu_id)
        # Distance between this RSU and the vehicle
        features['d_t_c'] = math.hypot(veh_node.pos[0] - self.pos[0], veh_node.pos[1] - self.pos[1])
        # Distance between the next RSU and the vehicle
        if next_rsu:
            dist_v_to_next = math.hypot(veh_node.pos[0] - next_rsu.pos[0], veh_node.pos[1] - next_rsu.pos[1])
            dist_v_to_cur = math.hypot(veh_node.pos[0] - self.pos[0], veh_node.pos[1] - self.pos[1])
            dist_cur_to_next = math.hypot(self.pos[0] - next_rsu.pos[0], self.pos[1] - next_rsu.pos[1])
            if dir_flag == -1:
                features['d_t_n'] = dist_v_to_next
            else:
                features['d_t_n'] = dist_v_to_cur + dist_cur_to_next
        else:
            features['d_t_n'] = float('inf')
        features['v_c_a'] = self.GetAvgSpeed()
        if next_rsu:
            features['v_n_a'] = next_rsu.GetAvgSpeed()
        else:
            features['v_n_a'] = -1.0
        features['tls_c'] = self._map_signal_state(net.GetSignalState(self.id))
        features['tls_n'] = self._map_signal_state(net.GetSignalState(next_rsu_id)) if next_rsu_id else 0
        # Time to next signal change
        features['tlt_c'] = net.GetSignalChangeTime(self.id)
        features['tlt_n'] = net.GetSignalChangeTime(next_rsu_id) if next_rsu_id else 0.0
        # Number of vehicles within current and next RSUs
        features['n_cur'] = len(self.GetVehiclesInRange())
        features['n_nxt'] = len(next_rsu.GetVehiclesInRange()) if next_rsu else 0
        return features

    def _map_signal_state(self, state: float) -> int:
        if state == 1.0: return -1
        if state == 2.0: return 0
        if state == 3.0: return 1
        return -2

    def _count_vehicles_to_next(self, next_rsu_id: Optional[str]) -> int:
        if not next_rsu_id:
            return 0
        count = 0
        for vid in self.GetVehiclesInRange():
            try:
                nxt = net.GetNextRSU(vid)
                if nxt == next_rsu_id:
                    count += 1
            except Exception:
                continue
        return count

    def _count_vehicles_in_range_to_next(self, rsu_node: net.Node, next_rsu_id: Optional[str]) -> int:
        if not next_rsu_id:
            return -1
        count = 0
        for vid in rsu_node.GetVehiclesInRange():
            try:
                nxt = net.GetNextRSU(vid)
                if nxt == next_rsu_id:
                    count += 1
            except Exception:
                continue
        return count
    
    def update_dwell(self, current_time: float) -> None:
        current_in_range = set(self.GetVehiclesInRange())
        for vid in current_in_range:
            if vid not in self._dwell_map:
                self._dwell_map[vid] = current_time
        for vid in list(self._dwell_map.keys()):
            if vid not in current_in_range:
                start_time = self._dwell_map.pop(vid)
                dwell_time = current_time - start_time
                self.dwell_queue.append(dwell_time)

###################################### Simulation Entry ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()