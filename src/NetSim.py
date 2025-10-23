from __future__ import annotations
import heapq, itertools, os, sumolib
os.environ.setdefault("SUMO_USE_LIBSUMO", "1")
import libsumo as sumo
from math import hypot
from collections import deque
from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Callable, List, Optional, Tuple, Dict
import xml.etree.ElementTree as ET
import src.Communications as comm
import src.sumo.make_sumo_set as sumo_set
import cv2, time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import builtins, traceback

def _node_alive(sim, node) -> bool:
    try: return any(n is node for n in getattr(sim, "nodes", []))
    except Exception: return False

b_step_log = True
b_reroute = False
MAX_EPISODE = 3
video_enabled: bool = False
recorder = None
USE_LIBSUMO = 1

MODE = False

@dataclass
class Packet:
    pkt_type: PacketType
    src: "Node"
    dst: "Node"
    payload: Any = None
    size_bytes: int = 1024

    frag_id: Optional[int] = None
    frag_index: int = 0
    frag_count: int = 1

def _fraction_inside_range(p0, p1, center, R) -> float:
    x0,y0 = p0; x1,y1 = p1; cx,cy = center
    d0 = hypot(x0-cx, y0-cy); d1 = hypot(x1-cx, y1-cy)
    if d0 <= R and d1 <= R: return 1.0
    if d0 > R and d1 > R: return 0.0
    lo, hi = 0.0, 1.0
    for _ in range(20):
        mid = (lo+hi)*0.5
        xm = x0 + mid*(x1-x0); ym = y0 + mid*(y1-y0)
        dm = hypot(xm-cx, ym-cy)
        if d0 <= R:
            if dm <= R: lo = mid
            else: hi = mid
        else:
            if dm <= R: hi = mid
            else: lo = mid
    return lo if (d0 <= R and d1 > R) else 0.0

class PacketType(Enum):
    HELLO   = auto()
    REQUEST = auto()
    DATA    = auto()
    ACK     = auto()
    PRECACHE= auto()
    REPORT  = auto()

class Node:
    def __init__(self, node_id: str, pos: Tuple[float, float] = (0.0, 0.0), comm_range: float = 0.0) -> None:
        self.id: str = node_id
        self.pos: Tuple[float, float] = pos
        self.prev_pos: Tuple[float, float] = pos
        self.sim: Optional[EventSimulator] = None
        self.comm_range: float = comm_range
        self.is_rsu: bool = False
        self.is_server: bool = False
        self.frag_buffers: Dict[int, Dict[int, Packet]] = {}
        self.dwell_queue: deque[float] = deque(maxlen=1000)
        self.speed_queue: deque[float] = deque(maxlen=1000)
        self._stream_sessions: Dict[str, Dict[str, float]] = {}

    def distance_to(self, other: "Node") -> float:
        try:
            x1, y1 = self.pos
            x2, y2 = other.pos
        except Exception as e:
            print(f"Error000: {e}"); traceback.print_exc()
        finally:
            return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5

    def send_packet(self, pkt: Packet, wave_rate_mbps: float = 6.0) -> None:
        if self.sim is None: raise RuntimeError("Node must be added to a simulator before sending packets")
        if pkt.size_bytes > comm.MAX_FRAME_SIZE:
            total_frags = (pkt.size_bytes + comm.MAX_FRAME_SIZE - 1) // comm.MAX_FRAME_SIZE
            if pkt.size_bytes >= comm.STREAM_THRESHOLD or total_frags > comm.FRAG_LIMIT:
                if getattr(self, "is_rsu", False) and getattr(pkt.dst, "is_rsu", False): medium = "fiber-1hop"
                elif (getattr(self, "is_rsu", False) and getattr(pkt.dst, "is_server", False)) or \
                     (getattr(self, "is_server", False) and getattr(pkt.dst, "is_rsu", False)):
                    medium = "fiber-13hop"
                else: medium = "wave"
                self._send_streaming(pkt, medium)
                return
        fragments = self._fragment_packet(pkt)
        for frag_pkt in fragments:
            if getattr(self, "is_rsu", False) and getattr(frag_pkt.dst, "is_rsu", False):
                distance = self.distance_to(frag_pkt.dst)
                data_rate_bps = comm.fiber_data_rate()
                trans_delay = (frag_pkt.size_bytes * 8) / data_rate_bps
                prop_delay = comm.fiber_propagation_delay(distance)
                arrival_time = self.sim.current_time + trans_delay + prop_delay
                self.sim.schedule_event(arrival_time, frag_pkt.dst.receive_packet, frag_pkt)
                continue
            if getattr(self, "is_rsu", False) and getattr(frag_pkt.dst, "is_server", False):
                distance = self.distance_to(frag_pkt.dst)
                data_rate_bps = comm.fiber_data_rate()
                trans_delay = (frag_pkt.size_bytes * 8) / data_rate_bps
                prop_delay = comm.fiber_propagation_delay(distance) * 13
                arrival_time = self.sim.current_time + trans_delay + prop_delay
                self.sim.schedule_event(arrival_time, frag_pkt.dst.receive_packet, frag_pkt)
                continue
            distance = self.distance_to(frag_pkt.dst)
            ch_idx, rate_mbps = comm.wave_channel_manager.allocate()
            data_rate_bps = comm.wave_data_rate(rate_mbps)
            trans_delay = comm.wave_transmission_delay(frag_pkt.size_bytes, data_rate_bps)
            prop_delay = comm.wave_propagation_delay(distance)
            arrival_time = self.sim.current_time + trans_delay + prop_delay

            def deliver(dst_node: "Node" = frag_pkt.dst, packet: Packet = frag_pkt, channel_idx: int = ch_idx) -> None:
                comm.wave_channel_manager.release(channel_idx)
                dst_node.receive_packet(packet)

            self.sim.schedule_event(arrival_time, deliver)

    def update_dwell(self, current_time: float) -> None:
        return
    
    def send_direct(self, pkt: Packet, wave_rate_mbps: float = 6.0) -> None:
        assert pkt.dst is not None and pkt.src is not None, "pkt.src/dst는 Node 객체여야 합니다."
        assert self.sim is not None and pkt.dst.sim is not None, "양쪽 노드가 시뮬레이터에 등록되어 있어야 함."
        arrival = self.sim.current_time + 0
        self.sim.schedule_event(arrival, pkt.dst.receive_packet, pkt)

    def _send_streaming(self, pkt: Packet, medium: str) -> None:
        assert self.sim is not None
        dst = pkt.dst
        distance = self.distance_to(dst)
        if medium == "wave":
            ch_idx, rate_mbps = comm.wave_channel_manager.allocate()
            data_rate_bps = comm.wave_data_rate(rate_mbps)
            prop_delay = comm.wave_propagation_delay(distance)
        elif medium == "fiber-1hop":
            data_rate_bps = comm.fiber_data_rate()
            prop_delay = comm.fiber_propagation_delay(distance)
            ch_idx = None
        elif medium == "fiber-13hop":
            data_rate_bps = comm.fiber_data_rate()
            prop_delay = comm.fiber_propagation_delay(distance) * 13
            ch_idx = None
        else:
            ch_idx, rate_mbps = comm.wave_channel_manager.allocate()
            data_rate_bps = comm.wave_data_rate(rate_mbps)
            prop_delay = comm.wave_propagation_delay(distance)
            medium = "wave"
        step_duration = self.sim.step
        bytes_per_step = max(1, int(data_rate_bps * step_duration / 8))
        bytes_left = pkt.size_bytes
        bytes_delivered = 0
        track_session = False
        sess_key = None
        if getattr(self, "is_rsu", False) and not getattr(dst, "is_rsu", False) and not getattr(dst, "is_server", False):
            track_session = True
            sess_key = dst.id
            self._stream_sessions[sess_key] = {"start_time": self.sim.current_time + prop_delay, "bytes": 0.0}

        def deliver_step() -> None:
            nonlocal bytes_left, bytes_delivered
            if medium == "wave":
                max_range = max(self.comm_range, dst.comm_range)
                current_distance = self.distance_to(dst)
                if current_distance > max_range and bytes_left > 0:
                        r = _fraction_inside_range(getattr(dst, "prev_pos", dst.pos), dst.pos, self.pos, max_range)
                        extra_time = r * step_duration
                        extra_bytes = min(bytes_left, int((data_rate_bps * extra_time) / 8))
                        if extra_bytes > 0:
                            bytes_left -= extra_bytes
                            bytes_delivered += extra_bytes
                            if track_session and sess_key is not None:
                                sess = self._stream_sessions.get(sess_key)
                                if sess is not None: sess["bytes"] = sess.get("bytes", 0.0) + extra_bytes
                        if ch_idx is not None: comm.wave_channel_manager.release(ch_idx)
                        if track_session and sess_key is not None:
                            sess = self._stream_sessions.pop(sess_key, None)
                            if sess:
                                dwell_time = max(self.sim.current_time - sess.get("start_time", self.sim.current_time), 0.0)
                                if dwell_time <= 0.0: dwell_time = self.sim.step
                                bytes_total = sess.get("bytes", 0.0)
                                self.speed_queue.append(((bytes_total * 8.0) / dwell_time) / 1e6 if dwell_time > 0 else 0.0)
                        if bytes_delivered > 0:
                            partial_pkt = Packet(pkt_type=pkt.pkt_type, src=pkt.src, dst=dst, payload=pkt.payload, size_bytes=bytes_delivered)
                            dst.receive_packet(partial_pkt, flag=1)
                        return
            send_now = bytes_per_step if bytes_left > bytes_per_step else bytes_left
            bytes_left -= send_now
            bytes_delivered += send_now
            if track_session and sess_key is not None:
                sess = self._stream_sessions.get(sess_key)
                if sess is not None: sess["bytes"] = sess.get("bytes", 0.0) + send_now
            if bytes_left <= 0:
                if medium == "wave" and ch_idx is not None: comm.wave_channel_manager.release(ch_idx)
                if track_session and sess_key is not None:
                    sess = self._stream_sessions.pop(sess_key, None)
                    if sess:
                        dwell_time = max(self.sim.current_time - sess.get("start_time", self.sim.current_time), 0.0) or self.sim.step
                        bytes_total = sess.get("bytes", 0.0)
                        self.dwell_queue.append(dwell_time)
                        self.speed_queue.append(((bytes_total * 8.0) / dwell_time) / 1e6 if dwell_time > 0 else 0.0)
                dst.receive_packet(pkt)
                return
            self.sim.schedule_event(self.sim.current_time + self.sim.step, deliver_step)
        self.sim.schedule_event(self.sim.current_time + prop_delay, deliver_step)

    def receive_packet(self, pkt: Packet, flag=0) -> None:
        assert self.sim is not None
        if video_enabled and recorder is not None and pkt is not None and pkt.src is not None:
            recorder.record_message(pkt, pkt.src, self)

        if pkt is None or pkt.src is None: return
        if self.sim is None or pkt.src.sim is None: return
        if not _node_alive(self.sim, self): return
        if not _node_alive(pkt.src.sim, pkt.src): return
        try:
            _sx, _sy = self.pos
            _dx, _dy = pkt.src.pos
        except Exception as e:
            print(f"Error001: {e}"); traceback.print_exc()
            return

        comm_range = max(self.comm_range, pkt.src.comm_range)
        if flag == 0 and pkt.frag_id is not None and pkt.frag_count > 1:
            buf = self.frag_buffers.setdefault(pkt.frag_id, {})
            buf[pkt.frag_index] = pkt
            distance_now = self.distance_to(pkt.src)
            if (comm_range > 0.0 and distance_now > comm_range):
                total_size = sum(f.size_bytes for f in buf.values())
                partial_pkt = Packet(pkt_type=pkt.pkt_type, src=pkt.src, dst=self, payload=pkt.payload, size_bytes=total_size)
                del self.frag_buffers[pkt.frag_id]
                self.on_receive(partial_pkt)
                return
            if len(buf) == pkt.frag_count:
                ordered = [buf[i] for i in sorted(buf.keys())]
                total_size = sum(f.size_bytes for f in ordered)
                full_pkt = Packet(pkt_type=pkt.pkt_type, src=pkt.src, dst=self, payload=pkt.payload, size_bytes=total_size)
                del self.frag_buffers[pkt.frag_id]
                self.on_receive(full_pkt)
            return
        if flag == 1:
            buf = self.frag_buffers.setdefault(pkt.frag_id, {})
            buf[pkt.frag_index] = pkt
            total_size = sum(f.size_bytes for f in buf.values())
            partial_pkt = Packet(pkt_type=pkt.pkt_type, src=pkt.src, dst=self, payload=pkt.payload, size_bytes=total_size)
            del self.frag_buffers[pkt.frag_id]
            self.on_receive(partial_pkt)
            return
        self.on_receive(pkt)

    def _fragment_packet(self, pkt: Packet) -> List[Packet]:
        if pkt.frag_id is not None or pkt.size_bytes <= comm.MAX_FRAME_SIZE: return [pkt]
        max_size = comm.MAX_FRAME_SIZE
        total_frags = (pkt.size_bytes + max_size - 1) // max_size
        frag_id = self.sim.get_next_frag_id() if self.sim is not None else 0
        fragments: List[Packet] = []
        remaining = pkt.size_bytes
        for idx in range(int(total_frags)):
            frag_size = max_size if remaining > max_size else remaining
            remaining -= frag_size
            frag_pkt = Packet(pkt_type=pkt.pkt_type, src=pkt.src, dst=pkt.dst, payload=pkt.payload, size_bytes=frag_size, frag_id=frag_id, frag_index=idx, frag_count=int(total_frags))
            fragments.append(frag_pkt)
        return fragments

    def on_receive(self, pkt: Packet) -> None:
        method_name = f"handle_{pkt.pkt_type.name.lower()}"
        handler = getattr(self, method_name, None)
        if callable(handler): handler(pkt)

    def broadcast(self, pkt_type: PacketType, payload: Any = None, size_bytes: int = 1024) -> None:
        if self.sim is None: return
        for node in self.sim.nodes:
            if node is self: continue
            max_range = builtins.max(getattr(self, "comm_range", 0.0), getattr(node, "comm_range", 0.0))
            if max_range <= 0.0: continue
            if self.distance_to(node) <= max_range:
                pkt = Packet(pkt_type=pkt_type, src=self, dst=node, payload=payload, size_bytes=size_bytes)
                self.send_packet(pkt)

    def finding_rsu(self, pkt_type: PacketType, payload: Any = None, size_bytes: int = 1024) -> None:
        if self.sim is None: return
        handler_name = f"handle_{pkt_type.name.lower()}"
        for node in self.sim.nodes:
            if not callable(getattr(node, handler_name, None)): continue
            if node is self: continue
            max_range = builtins.max(getattr(self, "comm_range", 0.0), getattr(node, "comm_range", 0.0))
            if max_range <= 0.0: continue
            if self.distance_to(node) <= max_range:
                pkt = Packet(pkt_type=pkt_type, src=self, dst=node, payload=payload, size_bytes=size_bytes)
                self.send_packet(pkt)

    def GetAvgSpeed(self) -> float:
        if not self.dwell_queue: return 0.0
        speeds_kmh: list[float] = []
        for dwell in list(self.dwell_queue):
            if dwell > 0 and self.comm_range > 0:
                speed_mps = (2.0 * self.comm_range) / dwell
                speeds_kmh.append(speed_mps * 3.6)
        return sum(speeds_kmh) / len(speeds_kmh) if speeds_kmh else 0.0

    def GetAvgRate(self) -> float:
        return (sum(self.speed_queue) / len(self.speed_queue)) if self.speed_queue else 54.0 / 8

    def GetVehiclesInRange(self) -> List[str]:
        if self.sim is None: return []
        nearby: List[str] = []
        for node in self.sim.nodes:
            if node is self: continue
            if getattr(node, "is_rsu", False) or getattr(node, "is_server", False): continue
            max_range = builtins.max(getattr(self, "comm_range", 0.0), getattr(node, "comm_range", 0.0))
            if max_range <= 0.0: continue
            if self.distance_to(node) <= max_range:
                nearby.append(node.id)
        return nearby
    
    def at_created(self) -> None: pass

class VideoRecorder:
    def __init__(self, net_file: str, output_path: str = "simulation.mp4", canvas_size: int = 1000) -> None:
        self.net_file = net_file
        self.canvas_size = canvas_size
        self.min_x = 0.0
        self.min_y = 0.0
        self.max_x = 1.0
        self.max_y = 1.0
        try:
            tree = ET.parse(self.net_file)
            root = tree.getroot()
            loc = root.find("location")
            if loc is not None:
                conv = loc.get("convBoundary")
                if conv:
                    parts = conv.split(",")
                    if len(parts) == 4:
                        self.min_x = float(parts[0])
                        self.min_y = float(parts[1])
                        self.max_x = float(parts[2])
                        self.max_y = float(parts[3])
        except Exception as e:
            print(f"Error002: {e}"); traceback.print_exc()
            self.min_x, self.min_y, self.max_x, self.max_y = 0.0, 0.0, 1.0, 1.0
        width = self.max_x - self.min_x
        height = self.max_y - self.min_y
        self.scale = float(self.canvas_size) / max(width if width > 0 else 1.0, height if height > 0 else 1.0)
        self.road_polylines: list[list[Tuple[float, float]]] = []
        try:
            tree = ET.parse(self.net_file)
            root = tree.getroot()
            for edge in root.findall("edge"):
                for lane in edge.findall("lane"):
                    shape = lane.get("shape")
                    if shape:
                        pts: list[Tuple[float, float]] = []
                        for token in shape.strip().split(" "):
                            if not token:
                                continue
                            try:
                                x_str, y_str = token.split(",")
                                pts.append((float(x_str), float(y_str)))
                            except Exception as e: print(f"Error003: {e}"); traceback.print_exc()
                        if len(pts) >= 2:
                            self.road_polylines.append(pts)
        except Exception as e:
            print(f"Error004: {e}"); traceback.print_exc()
            self.road_polylines = []
        if Image is not None:
            self.background = Image.new("RGB", (self.canvas_size, self.canvas_size), (0, 0, 0))
            draw = ImageDraw.Draw(self.background)
            for poly in self.road_polylines:
                if len(poly) < 2:
                    continue
                pix_pts = [self.world_to_pixel(pt) for pt in poly]
                flat_pts = []
                for p in pix_pts:
                    flat_pts.append(p)
                draw.line(flat_pts, fill=(80, 80, 80), width=1)
        else:
            self.background = None
        self.fps = 10
        self.frames_per_event = max(1, int(0.3 * self.fps + 0.5))
        if cv2 is not None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(output_path, fourcc, float(self.fps), (self.canvas_size, self.canvas_size))
        else:
            self.writer = None
        self.messages: list[Dict[str, Any]] = []
        if ImageFont is not None:
            try:
                self.font = ImageFont.load_default()
            except Exception as e:
                print(f"Error005: {e}"); traceback.print_exc()
                self.font = None
        else:
            self.font = None
        self.packet_colours: Dict[str, Tuple[int, int, int]] = {
            'HELLO': (200, 150, 255),      # pastel purple
            'REQUEST': (0, 255, 255),      # yellow/cyan mixture
            'DATA': (0, 255, 0),           # green
            'ACK': (0, 0, 255),            # red
            'PRECACHE': (255, 0, 255),     # magenta
        }

    def world_to_pixel(self, pos: Tuple[float, float]) -> Tuple[int, int]:
        x, y = pos
        px = int((x - self.min_x) * self.scale)
        py = int((self.max_y - y) * self.scale)
        px = max(0, min(self.canvas_size - 1, px))
        py = max(0, min(self.canvas_size - 1, py))
        return (px, py)

    def record_message(self, pkt: Packet, src_node: Node, dst_node: Node) -> None:
        if pkt is None or src_node is None or dst_node is None:
            return
        ptype_name = pkt.pkt_type.name if pkt.pkt_type else 'DATA'
        colour = self.packet_colours.get(ptype_name, (255, 255, 255))
        src_px = self.world_to_pixel(src_node.pos)
        dst_px = self.world_to_pixel(dst_node.pos)
        self.messages.append({'src': src_px, 'dst': dst_px, 'colour': colour, 'life': self.frames_per_event})

    def _draw_arrow(self, draw: ImageDraw.ImageDraw, src: Tuple[int, int], dst: Tuple[int, int], colour: Tuple[int, int, int]) -> None:
        draw.line([src, dst], fill=colour, width=1)
        sx, sy = src
        dx, dy = dst
        vx = dx - sx
        vy = dy - sy
        dist = (vx**2 + vy**2) ** 0.5
        if dist <= 0.0:
            return
        ux = vx / dist
        uy = vy / dist
        size = 5
        left = (-uy, ux)
        right = (uy, -ux)
        p1 = (int(dx - ux * size + left[0] * size), int(dy - uy * size + left[1] * size))
        p2 = (int(dx - ux * size + right[0] * size), int(dy - uy * size + right[1] * size))
        draw.polygon([dst, p1, p2], fill=colour)

    def record_frame(self, vehicles_dict: Dict[str, Node], rsu_nodes: List[Node], sim_time: float) -> None:
        if self.background is None or self.writer is None:
            return
        img = self.background.copy()
        draw = ImageDraw.Draw(img)
        for rsu in rsu_nodes:
            if getattr(rsu, "pos", None) is None:
                continue
            px, py = self.world_to_pixel(rsu.pos)
            radius = int(getattr(rsu, "comm_range", 0.0) * self.scale)
            if radius <= 0:
                continue
            bbox = (px - radius, py - radius, px + radius, py + radius)
            draw.ellipse(bbox, outline=(0, 128, 255))
        for vid, node in vehicles_dict.items():
            if getattr(node, "pos", None) is None:
                continue
            px, py = self.world_to_pixel(node.pos)
            colour = (255, 255, 255)
            radius = 3
            if hasattr(node, "mode"):
                if node.mode is not None: radius = 6
                if node.mode == "REQUEST": colour = (255, 0, 0)
                elif node.mode == "DOWNLOAD": colour = (0, 255, 0)
            draw.ellipse((px - radius, py - radius, px + radius, py + radius), fill=colour)
        remaining: list[Dict[str, Any]] = []
        for msg in self.messages:
            src_px = msg.get('src')
            dst_px = msg.get('dst')
            colour = msg.get('colour', (255, 255, 255))
            life = msg.get('life', 0)
            if src_px and dst_px and life > 0:
                self._draw_arrow(draw, src_px, dst_px, colour)
                if life - 1 > 0:
                    remaining.append({'src': src_px, 'dst': dst_px, 'colour': colour, 'life': life - 1})
        self.messages = remaining
        if self.font is not None:
            time_text = f"t={sim_time:.1f}s"
            draw.text((10, 10), time_text, fill=(255, 255, 255), font=self.font)
        frame = cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
        for _ in range(self.frames_per_event):
            self.writer.write(frame)

    def release(self) -> None:
        try:
            if self.writer is not None:
                self.writer.release()
        except Exception as e: print(f"Error006: {e}"); traceback.print_exc()

class Event:
    def __init__(self, time: float, handler: Callable, *args: Any, **kwargs: Any) -> None:
        self.time = time
        self.handler = handler
        self.args = args
        self.kwargs = kwargs

    def __lt__(self, other: "Event") -> bool: return self.time < other.time

class EventSimulator:
    def __init__(self, step: float = 1.0, max_time: float = 100.0) -> None:
        self.step = step
        self.max_time = max_time
        self.current_time: float = 0.0
        self._event_queue: List[Tuple[float, int, Event]] = []
        self._counter = itertools.count()
        self.nodes: List[Node] = []
        self._frag_counter = itertools.count(1)

    def add_node(self, node: Node) -> None:
        node.sim = self; self.nodes.append(node)
        hook = getattr(node, "at_created", None)
        if callable(hook): hook()

    def get_next_frag_id(self) -> int: return next(self._frag_counter)

    def schedule_event(self, time: float, handler: Callable, *args: Any, **kwargs: Any) -> None:
        if time > self.max_time: return
        count = next(self._counter)
        heapq.heappush(self._event_queue, (time, count, Event(time, handler, *args, **kwargs)))

    def _step_event(self) -> None:
        next_time = round(self.current_time + self.step ,6)
        self.schedule_event(next_time, self._step_event)

    def run(self) -> None:
        self.schedule_event(0.0, self._step_event)
        while self._event_queue:
            try:
                t, _, event = heapq.heappop(self._event_queue)
                if t > self.max_time: break
                if b_step_log: print(f"[{t:.3f}s] ▶ 이벤트 발생: {event.handler.__name__}({event.args}, {event.kwargs})")
                self.current_time = t
                event.handler(*event.args, **event.kwargs)
                if video_enabled and recorder is not None:
                    recorder.record_frame(vehicles, rsu_list, self.current_time)
                    
                time.sleep(0.005)
            except Exception as e: print(f"Error007: {e}"); traceback.print_exc()
        print("시뮬레이션 종료")

# =====================================================================
RSU_LIST: List[RSUNode] = []
RSU_DICT: Dict[str, RSUNode] = {}
sim = None
BASE_DIR = os.path.dirname(__file__)
NET_FILE = os.path.join(BASE_DIR, "sumo", "generated.net.xml")
NOD_DIR = os.path.join(BASE_DIR, "sumo", "generated.nod.xml")
RSU_DIR = os.path.join(BASE_DIR, "sumo", "rsu.poi.xml")
CFG_DIR = os.path.join(BASE_DIR, "sumo", "generated.sumocfg")

class VehicleNode(Node):
    def handle_request(self, pkt: Packet) -> None: pass

class RSUNode(Node):
    def handle_request(self, pkt: Packet) -> None: pass

import tkinter as tk
from tkinter import ttk
from typing import Callable, Dict, List

def launch_simulation() -> None:
    root = tk.Tk()
    root.title("SumoNetSim Configuration")

    rsu_range_var = tk.DoubleVar(value=800.0)
    max_episode_var = tk.IntVar(value=1)
    max_step_var = tk.IntVar(value=3600.0)
    avg_speed_var = tk.DoubleVar(value=0.0)
    outage_zone_var = tk.DoubleVar(value=800.0)
    num_blocks_var = tk.IntVar(value=5)
    density_var = tk.DoubleVar(value=0.0)
    reroute_var = tk.BooleanVar(value=False)
    step_log_var = tk.BooleanVar(value=True)
    video_var = tk.BooleanVar(value=False)

    i = 0
    ttk.Label(root, text="Number of episodes").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=max_episode_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Average vehicle speed (km/h)").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=avg_speed_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Vehicle density (/1km-lane)").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=density_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Grid size (number of RSUs along one axis)").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=num_blocks_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="RSU communication range (m)").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=rsu_range_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Max Steps").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=max_step_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Outage zone length (m)").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Entry(root, textvariable=outage_zone_var).grid(row=i, column=1, sticky="we", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Rerouting enabled").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Checkbutton(root, variable=reroute_var).grid(row=i, column=1, sticky="w", padx=5, pady=5)
    i += 1
    ttk.Label(root, text="Step Log").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Checkbutton(root, variable=step_log_var).grid(row=i, column=1, sticky="w", padx=5, pady=5)
    i += 1

    ttk.Label(root, text="Record simulation as video").grid(row=i, column=0, sticky="w", padx=5, pady=5)
    ttk.Checkbutton(root, variable=video_var).grid(row=i, column=1, sticky="w", padx=5, pady=5)
    i += 1

    root.columnconfigure(1, weight=1)

    def on_start() -> None:
        global MODE, MAX_EPISODE, b_reroute, b_step_log
        global video_enabled
        sumo_set.RSU_RANGE = rsu_range_var.get()
        MAX_EPISODE = max_episode_var.get()
        sumo_set.MAX_STEPS = max_step_var.get()
        sumo_set.OUTAGE_ZONE = outage_zone_var.get()
        sumo_set.NUM_BLOCKS = num_blocks_var.get()
        sumo_set.AV_SPEED = avg_speed_var.get()
        sumo_set.DENSITY = density_var.get()
        sumo_set.P_GEN = (sumo_set.DENSITY * sumo_set.SPEED) / 3600.0
        b_reroute = reroute_var.get()
        b_step_log = step_log_var.get()
        video_enabled = video_var.get()
        root.destroy()

    ttk.Button(root, text="Start Simulation", command=on_start).grid(row=i, column=0, columnspan=2, pady=10)
    root.mainloop()

vehicles: Dict[str, Node] = {}
rsu_list: List[Node] = []
rsu_dict: Dict[str, Node] = {}
sumo_cmd = ""
VehicleClass = None
RSUClass = None
st = 0

def InitSumoNetSim(VehicleClass: type = None, RSUClass: type = None,) -> None:
    launch_simulation()
    global MODE, CFG_DIR, sumo_cmd
    VehicleClass = VehicleClass or VehicleNode
    RSUClass = RSUClass or RSUNode

class SumoNetSim():
    def __init__(self, VehicleClass: type = VehicleNode, RSUClass: type = RSUNode,
                 start_message_fn: Callable[[EventSimulator, Dict[str, Node], List[Node], float], None] = None) -> None:
        global vehicles, rsu_list, rsu_dict
        self.sim = EventSimulator(step=1.0, max_time=sumo_set.MAX_STEPS)
        vehicles, rsu_list, rsu_dict = {}, [], {}
        self.VehicleClass = VehicleClass
        self.RSUClass = RSUClass
        self.start_message_fn = start_message_fn
        nod_file = NOD_DIR
        tree = ET.parse(nod_file)
        root = tree.getroot()
        self.b_init = False
        for node in root.findall("node"):
            if node.get("type") == "traffic_light":
                rsu_id = node.get("id")
                x = float(node.get("x"))
                y = float(node.get("y"))
                rsu_node = RSUClass(rsu_id, pos=(x, y))
                setattr(rsu_node, "is_rsu", True)
                rsu_dict[rsu_id] = rsu_node
                rsu_list.append(rsu_node)
                self.sim.add_node(rsu_node)

        rsu_positions = [(n.id, n.pos[0], n.pos[1]) for n in rsu_list]
        def uniq_sorted(vals, eps=1e-3):
            vals = sorted(vals)
            out = []
            for v in vals:
                if not out or abs(v - out[-1]) > eps:
                    out.append(v)
            return out
        uniq_x = uniq_sorted([x for _, x, _ in rsu_positions])
        uniq_y = uniq_sorted([y for _, _, y in rsu_positions])
        def idx_of(v, arr):
            best_i, best_d = 0, float('inf')
            for i, a in enumerate(arr):
                d = abs(v - a)
                if d < best_d:
                    best_d, best_i = d, i
            return best_i
        self.rsu_grid = {}
        for n in rsu_list:
            x, y = n.pos
            c = idx_of(x, uniq_x)
            r = idx_of(y, uniq_y)
            setattr(n, "rsu_row", r)
            setattr(n, "rsu_col", c)
            self.rsu_grid[(r, c)] = n

    def run(self) -> None:
        global recorder, video_enabled
        for ep in range(MAX_EPISODE):
            print(f"Episode {ep + 1}/{MAX_EPISODE}")
            if sumo_set.AV_SPEED == 0 or sumo_set.DENSITY == 0: self.b_init = False
            if not self.b_init:
                self.b_init = True
                sumo_set.make_sumo_files()
                tree = ET.parse(NOD_DIR)
                root = tree.getroot()
                with open(RSU_DIR, "w") as f:
                    f.write('<additional>\n')
                    for node in root.findall("node"):
                        if node.get("type") == "traffic_light":
                            rsu_id = node.get("id")
                            x = float(node.get("x"))
                            y = float(node.get("y"))
                            f.write(f'  <poi id="{rsu_id}" x="{x}" y="{y}" type="RSU" color="1,0,0"/>\n')
                    f.write('</additional>\n')
            base_args = [
                "sumo",
                "-c", CFG_DIR,
                "--step-length", "1.0",
                "--no-step-log", "true",
                "--duration-log.disable", "true",
                "--quit-on-end", "false",
                "--time-to-teleport", "60",
                "--collision.action", "warn",
            ]
            cmd = base_args
            sumo.start(cmd)

            if video_enabled:
                out_name = f"simulation_episode{ep+1}.mp4" if MAX_EPISODE > 1 else "simulation.mp4"
                net_path = NET_FILE
                recorder = VideoRecorder(net_file=net_path, output_path=out_name, canvas_size=1000)
            vehicle_rsu_states: Dict[str, set[str]] = {}

            def step_event() -> None:
                try:
                    time.sleep(0.005)
                    sumo.simulationStep()
                    self.sim.current_time = sumo.simulation.getTime()
                    current_ids = set(sumo.vehicle.getIDList())
                    
                    for vid in current_ids - set(vehicles.keys()):
                        x, y = sumo.vehicle.getPosition(vid)
                        node = self.VehicleClass(vid, pos=(x, y))
                        vehicles[vid] = node
                        self.sim.add_node(node)
                    for vid in current_ids:
                        node = vehicles.get(vid)
                        if node:
                            try: x, y = sumo.vehicle.getPosition(vid)
                            except Exception as e: print(f"Error008: {e}"); traceback.print_exc()
                            node.pos = (x, y)

                    if b_reroute:
                        for vid in current_ids:
                            node = vehicles.get(vid)
                            if node is None: continue
                            inside_rs: set[str] = set()
                            for rsu in rsu_list:
                                try:
                                    if rsu.comm_range > 0.0 and node.distance_to(rsu) <= rsu.comm_range: inside_rs.add(rsu.id)
                                except Exception as e: print(f"Error009: {e}"); traceback.print_exc(); continue
                            prev_set = vehicle_rsu_states.get(vid, set())
                            newly_entered = inside_rs.difference(prev_set)
                            if newly_entered:
                                try: sumo.vehicle.rerouteTraveltime(vid)
                                except Exception as e: print(f"Error010: {e}"); traceback.print_exc()
                            vehicle_rsu_states[vid] = inside_rs
                    for vid in list(vehicles.keys() - current_ids): vehicles.pop(vid, None);  vehicle_rsu_states.pop(vid, None)
                    
                    for node in self.sim.nodes:
                        node.update_dwell(self.sim.current_time)
                except Exception as e: print(f"Error011: {e}"); traceback.print_exc()
                finally: self.sim.schedule_event(self.sim.current_time + self.sim.step, step_event)

            self.sim._step_event = step_event
            self.sim.schedule_event(0.0, self.sim._step_event)
            if self.start_message_fn: self.start_message_fn(self.sim, vehicles, rsu_list, sumo_set.T_to_INIT)
            self.sim.run()

            if video_enabled and recorder is not None: recorder.release()
            sumo.close()
            
_network_cache: Optional[sumolib.net.Net] = None

def GetSpeed(vehicle_id: str) -> float:
    try: return float(sumo.vehicle.getSpeed(vehicle_id))
    except Exception as e: print(f"Error012: {e}"); traceback.print_exc(); return 0.0

def GetAcceleration(vehicle_id: str) -> float:
    try: return float(sumo.vehicle.getAcceleration(vehicle_id))
    except Exception as e: print(f"Error013: {e}"); traceback.print_exc(); return 0.0

def GetPosition(vehicle_id: str) -> Tuple[float, float]:
    try:
        pos = sumo.vehicle.getPosition(vehicle_id)
        return (float(pos[0]), float(pos[1]))
    except Exception as e: print(f"Error014: {e}"); traceback.print_exc(); return (0.0, 0.0)

def GetRoutes(vehicle_id: str) -> List[str]:
    global _network_cache
    if _network_cache is None:
        try: _network_cache = sumolib.net.readNet(NET_FILE)
        except Exception as e: print(f"Error015: {e}"); _network_cache = None
    try: route_edges = sumo.vehicle.getRoute(vehicle_id)
    except Exception as e: print(f"Error016: {e}"); traceback.print_exc(); return []
    if not route_edges: return []
    rsu_ids: List[str] = []
    seen: set[str] = set()
    if _network_cache is None: return []
    for edge_id in route_edges:
        try: edge = _network_cache.getEdge(edge_id)
        except Exception as e: print(f"Error017: {e}"); traceback.print_exc(); continue
        for node in (edge.getFromNode(), edge.getToNode()):
            try: nid = node.getID()
            except Exception as e: print(f"Error018: {e}"); traceback.print_exc(); continue
            if nid in rsu_dict and nid not in seen: rsu_ids.append(nid); seen.add(nid)
    return rsu_ids

def GetNextRSU(vehicle_id: str) -> Optional[str]:
    global _network_cache
    if _network_cache is None:
        try: _network_cache = sumolib.net.readNet(NET_FILE)
        except Exception as e: print(f"Error019: {e}"); traceback.print_exc(); _network_cache = None
    try:
        route_edges = sumo.vehicle.getRoute(vehicle_id)
        idx = sumo.vehicle.getRouteIndex(vehicle_id)
    except Exception as e: print(f"Error020: {e}"); traceback.print_exc(); return None
    if not route_edges: return None
    if idx is None or idx < 0: idx = 0
    if _network_cache is None: return None
    veh_x,veh_y = sumo.vehicle.getPosition(vehicle_id)
    for pos in range(int(idx), len(route_edges)):
        edge_id = route_edges[pos]
        edge = _network_cache.getEdge(edge_id)
        nid = edge.getToNode().getID()
        rsu_x, rsu_y = rsu_dict[nid].pos
        if (rsu_x - veh_x) ** 2 + (rsu_y - veh_y) ** 2 <= rsu_dict[nid].comm_range ** 2:
            return _network_cache.getEdge(route_edges[pos+1]).getToNode().getID()
        else:
            return nid
    return None

def GetSignalState(rsu_id: str) -> float:
    try:
        state_str = sumo.trafficlight.getRedYellowGreenState(rsu_id)
        if not state_str: return 0.0
        c = state_str[0]
        mapping = {'R': 1.0, 'r': 1.0, 'Y': 2.0, 'y': 2.0, 'G': 3.0, 'g': 3.0}
        return mapping.get(c, 0.0)
    except Exception as e: print(f"Error021: {e}"); traceback.print_exc(); return 0.0

def GetSignalChangeTime(rsu_id: str) -> float:
    try:
        next_switch = sumo.trafficlight.getNextSwitch(rsu_id)
        current_time = sumo.simulation.getTime()
        return max(0.0, float(next_switch - current_time))
    except Exception as e: print(f"Error022: {e}"); traceback.print_exc(); return 0.0