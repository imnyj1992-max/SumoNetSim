from __future__ import annotations
import random
import src.NetSim as net
from typing import Dict, List, Tuple, Optional

# 이 예제는 V2V Precaching in Outage zone 논문을 기반으로 작성되었습니다.
# 해당 예제는 논문의 기초 개념을 시뮬레이션으로 구현한 것으로, 실제 논문과는 다소 차이가 있을 수 있습니다.

# 이 예제의 목적
# 1) 

######################## Config ########################
START_TIME    = 100.0
REQ_PERIOD    = 1.0
REQ_SIZE      = 64
REPLY_SIZE    = 32
CONTENT_ID    = 42
CONTENT_SIZE  = 1024 * 1024 * 1024  # 1 GB

######################## Start Hook ########################
def start_message(sim: net.EventSimulator, vehicles: Dict[str, net.Node], rsu_list: List[net.Node], t_init) -> None:
    """
    Called once when the SUMO simulation is initialised.  This hook adds the
    content server node to the simulator and schedules the first request
    generation on a randomly chosen vehicle.  Without this function the
    simulation would run but no vehicle would ever begin requesting the
    target content.

    Args:
        sim: The event simulator instance controlling asynchronous events.
        vehicles: A dictionary mapping vehicle identifiers to their Node
                  objects.  This dictionary is populated as vehicles enter
                  the simulation.
        rsu_list: A list of RSU Node objects currently in the simulation.
        t_init:  The initial simulation time (unused here).
    """
    # Ensure the content server is present in the simulation.  This call
    # attaches the server node to the event simulator so that messages
    # destined for it will be delivered correctly.
    sim.add_node(ContentServer())

    def kick_off() -> None:
        """Select a random vehicle and begin periodic REQUEST messages."""
        # If there are no vehicles yet, do nothing.  This might occur if
        # traffic generation is delayed or disabled in the SUMO scenario.
        if not vehicles:
            return
        # Choose a vehicle at random so that different runs of the
        # simulation exercise different topological situations.
        veh = random.choice(list(vehicles.values()))
        print(f"[{sim.current_time:.6f}s] Select target vehicle: {veh.id}. Start periodic REQUEST.")
        # Clear any prior mode and kick off the request cycle.
        veh.mode = None
        veh.start_request()

    # Schedule the initial kick‑off after START_TIME seconds of simulation
    # time.  At that point at least one vehicle should be present.
    sim.schedule_event(START_TIME, kick_off)

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
        
        # Cache to store precached content.  Maps content ID to the size of
        # content bytes that were cached.  We treat any non‑zero entry as
        # an indication that this vehicle has cached content available to
        # serve to requesting vehicles when in an outage zone.
        self.cache: Dict[int, int] = {}
        # When an RSU precaches content, it identifies which vehicle the
        # cached data is intended for.  We store that vehicle ID here so
        # that we can proactively deliver the cached content as soon as
        # both vehicles are within V2V communication range and outside
        # of any RSU’s coverage area.  Once delivery is attempted, this
        # field will be cleared to avoid repeated transmissions.
        self.req_veh_id: Optional[str] = None

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
        """
        Called when a vehicle receives an ACK packet.  This ACK indicates that
        either an RSU or another vehicle has accepted the request and that the
        requesting vehicle should stop sending further REQUEST messages.  The
        source of the ACK may be an RSU (is_rsu flag set) or another vehicle
        (neither is_rsu nor is_server).  Adjust the printed label accordingly.
        """
        # Enter download mode to prevent further REQUESTs until the current
        # download completes.  This mirrors the original behaviour in the sample
        # code, which treats any ACK as a trigger to stop REQUESTs.
        self.mode = "DOWNLOAD"
        # Build a human‑readable source label.  RSUs are prefixed with "R",
        # other vehicles with "V", and servers with "S".  This makes the
        # console output easier to interpret during debugging and demonstration.
        if getattr(pkt.src, "is_rsu", False):
            src_label = f"R{pkt.src.id}"
        elif getattr(pkt.src, "is_server", False):
            src_label = f"S{pkt.src.id}"
        else:
            src_label = f"V{pkt.src.id}"
        print(f"[{self.sim.current_time:.6f}s] REPLY [{src_label} → V{self.id}] to stop REQUEST")

    def handle_data(self, pkt: net.Packet) -> None:
        """
        Called when a DATA packet arrives.  Updates the counters for how
        many bytes have been received so far and resets the request state
        accordingly.  It also prints a summary of the transfer including
        whether the transfer is complete.  The label for the sender is
        determined using its type (RSU, server, or vehicle) for clarity.
        """
        # Accumulate received bytes and compute how many remain.
        self.bytes_received += pkt.size_bytes
        self.bytes_remained = max(0, CONTENT_SIZE - self.bytes_received)
        # Convert bytes to megabytes for user‑friendly output.
        mb = self.bytes_received / (1024 * 1024)
        # Measure dwell time (time spent downloading) since the last request.
        dwell = self.sim.current_time - (self._dwell_start or self.sim.current_time)
        # Clear current mode; downloading is a transient state.
        self.mode = None
        # Determine a label for the source for logging.  Prefix RSUs with
        # "R", servers with "S", and other vehicles with "V".
        if getattr(pkt.src, "is_rsu", False):
            src_label = f"R{pkt.src.id}"
        elif getattr(pkt.src, "is_server", False):
            src_label = f"S{pkt.src.id}"
        else:
            src_label = f"V{pkt.src.id}"
        # Provide additional context from the payload if available.
        from_str = pkt.payload.get("from") if isinstance(pkt.payload, dict) else None
        # Log whether the content transfer is complete or still in progress.
        if self.bytes_received >= CONTENT_SIZE:
            print(f"[{self.sim.current_time:.6f}s] DATA [{src_label} → V{self.id}] {CONTENT_ID} Received {mb:.2f} MB from {from_str or src_label} (Complete, Dwell {dwell:.3f}s)")
        else:
            print(f"[{self.sim.current_time:.6f}s] DATA [{src_label} → V{self.id}] {CONTENT_ID} Received {mb:.2f} MB from {from_str or src_label} (Progress, Dwell {dwell:.3f}s)")
            # Continue requesting the remaining content if not yet complete.
            self.start_request()
    
    def handle_precache(self, pkt: net.Packet) -> None:
        if self.b_log: print(f"[{self.sim.current_time:.6f}s] PRECACHE [R{pkt.src.id} → V{self.id}] c_id={CONTENT_ID}")
        p = pkt.payload
        # Store the size of the cached content.  If the RSU included a
        # specific size, honour it; otherwise assume the entire content
        # will be cached.  Additionally, record the ID of the vehicle
        # this cache is intended to serve so that we can proactively
        # deliver it in the outage zone.
        c_id = p.get("c_id")
        if c_id is not None:
            self.cache[c_id] = p.get("c_size", CONTENT_SIZE)
        # Record the target vehicle ID (if provided) so that we know who
        # to deliver to when out of RSU range.  Without this field the
        # vehicle will still respond to REQUEST messages via handle_request.
        tgt = p.get("veh_id")
        if isinstance(tgt, str):
            self.req_veh_id = tgt

    # 요청 패킷을 처리하는 메서드 추가
    # 다른 차량이 RSU의 통신 범위를 벗어난 경우, PRECACHE를 통해 받은 컨텐츠를 제공하기 위함
    # V2V 통신을 통해 cached content를 전달하고, 추가적으로 ACK를 보내 요청을 중지하도록 한다.
    def handle_request(self, pkt: net.Packet) -> None:
        """
        Handle a REQUEST packet sent from another vehicle.  If this vehicle has
        precached content and shares the same next RSU as the requester, it
        replies with an ACK to stop repeated REQUESTs and sends DATA with the
        cached content.  This allows V2V delivery of content in outage zones
        where RSUs are unavailable.
        """
        src = pkt.src  # 요청 차량
        # 자신이 RSU이거나 서버인 경우에는 처리하지 않음
        if getattr(self, "is_rsu", False) or getattr(self, "is_server", False):
            return
        # 동일 차량에게는 응답하지 않음
        if src is self:
            return
        # 다음 RSU가 다르면 응답하지 않음 (기본 방안: next RSU가 같은 차량만 precache)
        try:
            if net.GetNextRSU(self.id) != net.GetNextRSU(src.id):
                return
        except Exception:
            # GetNextRSU가 실패할 경우에도 응답하지 않음
            return
        # precached 콘텐츠가 없으면 응답하지 않음
        if CONTENT_ID not in self.cache:
            return
        # ACK 전송: 요청 차량의 REQUEST 반복을 중지시키기 위해
        ack = net.Packet(net.PacketType.ACK, self, src, payload={"reply": True}, size_bytes=REPLY_SIZE)
        self.send_packet(ack)
        # 요청 차량이 아직 받아야 할 바이트 수. getattr로 기본값을 CONTENT_SIZE로 처리
        remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
        # 우리 캐시에 저장된 크기와 비교하여 최소값을 전송한다
        cached_size = self.cache.get(CONTENT_ID, CONTENT_SIZE)
        send_size = min(remaining, cached_size)
        # DATA 전송
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA,
                                    src=self,
                                    dst=src,
                                    payload={"c_id": CONTENT_ID, "from": f"V{self.id} (cached)"},
                                    size_bytes=send_size))

    def update_dwell(self, current_time: float) -> None:
        """
        Periodically invoked by the simulator to allow nodes to perform
        housekeeping tasks.  We override this to implement proactive V2V
        delivery of precached content.  If this vehicle has cached
        content destined for a particular requesting vehicle, it will
        check whether that target vehicle is within V2V range and
        simultaneously outside of RSU coverage.  If so, an ACK and DATA
        packet are sent to the requester without waiting for an explicit
        REQUEST.  Once a delivery attempt is made, the target ID is
        cleared to avoid redundant transmissions.
        """
        # Call the base class implementation to maintain any existing
        # dwell time bookkeeping (even though it is currently a no‑op).
        super().update_dwell(current_time)
        # If we have no target or no cached data, there is nothing to do.
        if not self.req_veh_id:
            return
        if CONTENT_ID not in self.cache:
            return
        # Look up the target vehicle node.  The global vehicles dict maps
        # vehicle IDs to their Node objects.  If the target has left the
        # simulation, we drop the assignment.
        target = net.vehicles.get(self.req_veh_id)
        if target is None or target is self:
            self.req_veh_id = None
            return
        # Ensure both this vehicle and the target are in mutual V2V range.
        max_range = max(getattr(self, "comm_range", 0.0), getattr(target, "comm_range", 0.0))
        if max_range <= 0.0:
            return
        # Check that the distance between the two vehicles is within
        # communication range.  Without proximity we cannot deliver.
        if self.distance_to(target) > max_range:
            return
        # Determine whether the target vehicle is outside of any RSU’s
        # communication range.  We only proactively deliver in outage
        # zones because RSUs should serve when available.  An outage zone
        # is defined when no RSU is within its comm_range of the target.
        out_of_rsu = True
        for rsu in getattr(net, "rsu_list", []):
            try:
                # Skip None entries or non‑RSU nodes just in case.
                if rsu is None or not getattr(rsu, "is_rsu", False):
                    continue
                if target.distance_to(rsu) <= getattr(rsu, "comm_range", 0.0):
                    out_of_rsu = False
                    break
            except Exception:
                continue
        if not out_of_rsu:
            return
        # If the target is still downloading content (mode == "DOWNLOAD")
        # or has already completed, do not interrupt.  We only assist
        # vehicles that are actively requesting data.
        if getattr(target, "mode", None) not in ("REQUEST", None):
            return
        # Send an ACK to stop the target’s REQUEST messages.  This helps
        # reduce unnecessary broadcast traffic.
        ack = net.Packet(net.PacketType.ACK, self, target, payload={"reply": True}, size_bytes=REPLY_SIZE)
        self.send_packet(ack)
        # Calculate the number of bytes the target still needs and the
        # amount we have cached.  Only the minimum of these should be
        # delivered.
        remaining = max(0, getattr(target, "bytes_remained", CONTENT_SIZE))
        cached_size = self.cache.get(CONTENT_ID, 0)
        send_size = min(remaining, cached_size)
        if send_size <= 0:
            self.req_veh_id = None
            return
        # Send the DATA packet with a payload indicating that it came
        # from this vehicle’s cache.  The size_bytes field controls how
        # long the transmission will take in the simulator.
        data_pkt = net.Packet(pkt_type=net.PacketType.DATA,
                              src=self,
                              dst=target,
                              payload={"c_id": CONTENT_ID, "from": f"V{self.id} (cached)"},
                              size_bytes=send_size)
        self.send_packet(data_pkt)
        # Once a proactive delivery has been attempted, clear the
        # assignment to avoid duplicate transmissions on subsequent
        # simulation steps.  We do not decrement our cache; subsequent
        # requests will still be served on demand via handle_request().
        self.req_veh_id = None

        

class RSUNode(net.Node):
    def __init__(self, node_id: str, pos: Tuple[float, float]=(0.0, 0.0)) -> None:
        super().__init__(node_id, pos=pos, comm_range=800.0)
        self.is_rsu = True
        self.cache: Dict[int, int] = {}
        if random.random() < 0.3: self.cache[CONTENT_ID] = CONTENT_SIZE
        self._pending_vehicle = None

    def handle_request(self, pkt: net.Packet) -> None:
        src = pkt.src
        print(f"[{self.sim.current_time:.6f}s] REQUEST [V{src.id} → R{self.id}] c_id={CONTENT_ID}")
        ack = net.Packet(net.PacketType.ACK, self, src, payload={"reply": True}, size_bytes=REPLY_SIZE)
        self.send_packet(ack)

        if CONTENT_ID in self.cache:
            remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
            self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=src, payload={"c_id": CONTENT_ID, "from": f"RSU:{self.id} (cached)"}, size_bytes=remaining))
            self.VehicleSelection(src, remaining)
            return

        # If the RSU does not have the content cached, forward the request to the server
        server = next((n for n in self.sim.nodes if getattr(n, "is_server", False)), None)
        self._pending_vehicle = src
        remaining = max(0, getattr(src, "bytes_remained", CONTENT_SIZE))
        # Forward the request upstream along with the remaining size and requester ID
        self.send_packet(net.Packet(pkt_type=net.PacketType.REQUEST,
                                    src=self,
                                    dst=server,
                                    payload={"c_id_req": CONTENT_ID,
                                             "c_size": remaining,
                                             "veh_id": src.id,
                                             "from_rsu": self.id},
                                    size_bytes=REQ_SIZE))
        # Regardless of whether the RSU has cached content locally, pre‑cache the remaining
        # data to nearby vehicles whose next RSU matches that of the requester.  This
        # implementation defers to the VehicleSelection helper, which performs the
        # range and next‑RSU checks and sends the appropriate PRECACHE packets.
        self.VehicleSelection(src, remaining)

    def handle_data(self, pkt: net.Packet) -> None:
        if self._pending_vehicle is None: return
        print(f"[{self.sim.current_time:.6f}s] DATA [Server → R{self.id}] c_id={CONTENT_ID} from {pkt.src.id}")
        v = self._pending_vehicle
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=v, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=pkt.size_bytes,))
        self._pending_vehicle = None
        self.VehicleSelection(v, pkt.size_bytes)

    def VehicleSelection(self, src, remaining: int) -> None:        
        v_list = self.GetVehiclesInRange()
        for v in v_list:
            # v는 차량 ID 문자열
            if v == src.id:
                continue
            # 다음 RSU가 다른 차량은 건너뜀
            if net.GetNextRSU(v) != net.GetNextRSU(src.id):
                continue
            dst_node = net.vehicles.get(v)
            if dst_node is None:
                continue
            # PRECACHE 메시지를 전송
            self.send_packet(net.Packet(pkt_type=net.PacketType.PRECACHE,
                                        src=self,
                                        dst=dst_node,
                                        payload={"c_id": CONTENT_ID, "c_size": remaining, "veh_id": src.id},
                                        size_bytes=REQ_SIZE))

class ContentServer(net.Node):
    def __init__(self, node_id: str="SERVER", pos: Tuple[float, float]=(1e6, 1e6)) -> None:
        super().__init__(node_id, pos=pos, comm_range=0.0)
        self.is_server = True

    def handle_request(self, pkt: net.Packet) -> None:
        c_size = pkt.payload.get("c_size")
        print(f"[{self.sim.current_time:.6f}s] REQUEST [R{pkt.src.id} → Server] c_id={CONTENT_ID}")
        self.send_packet(net.Packet(pkt_type=net.PacketType.DATA, src=self, dst=pkt.src, payload={"c_id": CONTENT_ID, "from": "server"}, size_bytes=c_size))

###################################### 시뮬 시작 ######################################
if __name__ == "__main__":
    net.InitSumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode)
    netsim = net.SumoNetSim(VehicleClass=VehicleNode, RSUClass=RSUNode, start_message_fn=start_message)
    netsim.run()