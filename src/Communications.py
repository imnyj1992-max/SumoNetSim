# WAVE (802.11p)
# Fiber Link


# Constants
C_LIGHT = 3e8  # Speed of light in vacuum (m/s)
REFRACTIVE_INDEX_FIBER = 1.4682  # Typical refractive index for single-mode fiber
FIBER_PROPAGATION_SPEED = C_LIGHT / REFRACTIVE_INDEX_FIBER  # ≈2.044e8 m/s

# Maximum frame size in bytes for WAVE transmissions.  Larger packets will be
# split into multiple fragments of at most this size.  In vehicular content
# delivery scenarios such as streaming media, higher layer protocols often
# employ payload sizes around 8 KiB (8192 bytes) per IP packet.  Using a
# frame size of 8192 bytes better reflects typical application data units
# while still keeping fragment counts manageable.  If a packet exceeds
# this size, the simulator will either fragment it into multiple frames
# (up to FRAG_LIMIT) or switch to streaming mode for very large payloads.
MAX_FRAME_SIZE = 8192

# When a packet would be fragmented into more than this number of pieces or
# exceeds the streaming threshold below, the simulator will switch to a
# streaming transfer mode.  In streaming mode the sender meters data
# continuously each simulation step rather than scheduling a separate
# event per fragment.  This avoids scheduling tens of thousands of
# fragment events for very large payloads.
FRAG_LIMIT = 5000

# Any packet larger than this many bytes will automatically be streamed
# instead of fragmented.  This provides a hard cutoff so that extremely
# large transfers (e.g. tens of megabytes) do not overwhelm the event
# queue.  The default of one megabyte can be tuned up or down based on
# expected workloads.
STREAM_THRESHOLD = 1 * 1024 * 1024  # 1 MiB

# ---------------------------------------------------------------------------
# Wave channel management
#
# The WAVE (802.11p) physical layer uses multiple frequency channels.  Each
# channel can service a limited number of concurrent users before their
# effective data rate must be reduced.  A naive implementation would simply
# divide the raw throughput by the number of users, but real systems
# downshift to discrete modulation rates as more users compete.  The
# WaveChannelManager class encapsulates this behaviour.  Each channel keeps
# track of the number of active transmissions.  When a new transmission
# starts, the manager assigns it to the least‑loaded channel and returns the
# appropriate modulation rate (in Mbps) for that channel based on the number
# of concurrent users.  When the transmission completes the channel is
# released and subsequent transmissions can once again enjoy higher rates.

from typing import Tuple, Optional

# --- Communications.py ---

class WaveChannelManager:
    def __init__(self, num_channels: int = 4) -> None:
        self.num_channels = num_channels
        self.channels: list[list[object]] = [[] for _ in range(num_channels)]
        # 속도 단계 (Mbps)
        self.speeds = [27.0, 13.0, 6.0, 3.0]
        self.min_rate = 0.1           # 바닥 속도
        self.capacity = 100.0         # 채널 합계 상한 (Mbps)

    def _rate_per_user(self, users: int) -> float:
        if users <= 0:
            return 0.0
        if users <= len(self.speeds):
            return self.speeds[users - 1]
        # 초과 접속자: 마지막 단계(3Mbps)부터 반감 (오버플로우 방지용 클램프)
        extra = users - len(self.speeds)
        extra = min(extra, 30)  # <-- 여기서 2**extra 폭주 방지
        return max(self.speeds[-1] / (2.0 ** float(extra)), self.min_rate)

    def _sum_rate_if_add_one(self, users_now: int) -> tuple[float, float]:
        current_rate = self._rate_per_user(users_now)
        sum_now = users_now * current_rate
        add_rate = self._rate_per_user(users_now + 1)
        return sum_now, add_rate

    def allocate(self) -> tuple[int, float]:
        candidates = []
        for i in range(self.num_channels):
            u = len(self.channels[i])
            sum_now, add_rate = self._sum_rate_if_add_one(u)
            candidates.append((i, sum_now, add_rate))

        candidates.sort(key=lambda x: x[1])

        for i, sum_now, add_rate in candidates:
            if sum_now + add_rate <= self.capacity + 1e-9:
                self.channels[i].append(object())
                return i, add_rate

        i_min = min(range(self.num_channels), key=lambda k: len(self.channels[k]))
        self.channels[i_min].append(object())
        leftover = max(self.capacity - self._sum_rate_if_add_one(len(self.channels[i_min])-1)[0], 0.0)
        rate = max(self.min_rate, min(leftover, self.speeds[0]))
        return i_min, rate

    def release(self, ch_idx: int) -> None:
        if 0 <= ch_idx < self.num_channels and self.channels[ch_idx]:
            self.channels[ch_idx].pop()

wave_channel_manager: WaveChannelManager = WaveChannelManager()

# -------------------------- Delay Models --------------------------
def fiber_propagation_delay(distance_m: float, propagation_speed: float = FIBER_PROPAGATION_SPEED) -> float:
    return distance_m / propagation_speed


def wave_propagation_delay(distance_m: float, propagation_speed: float = C_LIGHT) -> float:
    return distance_m / propagation_speed

# -------------------------- Transmission Delay --------------------------
def wave_transmission_delay(frame_size_bytes: float, data_rate_bps: float) -> float:
    return (frame_size_bytes * 8) / data_rate_bps

# -------------------------- Composite Delay --------------------------
def wave_total_delay(distance_m: float,
                     frame_size_bytes: float,
                     data_rate_bps: float,
                     propagation_speed: float = C_LIGHT) -> float:
    return wave_propagation_delay(distance_m, propagation_speed) + \
           wave_transmission_delay(frame_size_bytes, data_rate_bps)

# -------------------------- Data Rate Models --------------------------
def fiber_data_rate(rate_gbps: float = 10.0) -> float:
    return rate_gbps * 1e9


def wave_data_rate(rate_mbps: float = 6.0) -> float:
    return rate_mbps * 1e6

# -------------------------- Throughput per Simulation Step --------------------------
def data_per_step(data_rate_bps: float,
                  step_duration_sec: float = 1.0) -> float:
    bits = data_rate_bps * step_duration_sec
    return bits / 8

# -------------------------- Adjusted Throughput Considering Delay --------------------------
def adjusted_bytes_per_step(distance_m: float,
                            data_rate_bps: float,
                            step_duration_sec: float = 1.0,
                            propagation_speed: float = C_LIGHT) -> float:
    prop_delay = distance_m / propagation_speed
    avail_time = step_duration_sec - prop_delay
    if avail_time <= 0:
        return 0.0
    return (data_rate_bps * avail_time) / 8

# -------------------------- Fiber Fetch with Hop Count --------------------------
def fiber_fetch_bytes_per_step(distance_m: float,
                                step_duration_sec: float = 1.0,
                                rate_gbps: float = 10.0,
                                hop_count: int = 1,
                                propagation_speed: float = FIBER_PROPAGATION_SPEED) -> float:
    data_rate = fiber_data_rate(rate_gbps)
    total_prop = hop_count * (distance_m / propagation_speed)
    avail_time = step_duration_sec - total_prop
    if avail_time <= 0:
        return 0.0
    return (data_rate * avail_time) / 8

# -------------------------- Relay Throughput Combining Links --------------------------
def relay_bytes_per_step(distance_wave_m: float,
                          distance_fiber_m: float,
                          step_duration_sec: float = 1.0,
                          wave_rate_mbps: float = 6.0,
                          fiber_rate_gbps: float = 10.0,
                          prop_speed_wave: float = C_LIGHT,
                          prop_speed_fiber: float = FIBER_PROPAGATION_SPEED) -> float:
    r_wave = wave_data_rate(wave_rate_mbps)
    r_fiber = fiber_data_rate(fiber_rate_gbps)
    eff_rate = (r_wave * r_fiber) / (r_wave + r_fiber)
    d_prop_wave = distance_wave_m / prop_speed_wave
    d_prop_fiber = distance_fiber_m / prop_speed_fiber
    total_delay = d_prop_wave + d_prop_fiber
    avail_time = step_duration_sec - total_delay
    if avail_time <= 0:
        return 0.0
    return (eff_rate * avail_time) / 8

# -------------------------- Example Usage --------------------------
if __name__ == '__main__':
    distance_fiber = 1000  # meters on fiber
    distance_wave = 100  # meters over WAVE

    print(f"Adjusted WAVE bytes:{adjusted_bytes_per_step(100, wave_data_rate() / 1024):.2f} kB = {adjusted_bytes_per_step(100, wave_data_rate() / 1024)/(1024):.2f} MB")
    print(f"Fiber fetch bytes (13 hops):{fiber_fetch_bytes_per_step(1000, hop_count=13)/(1024**2):.2f} MB\t {fiber_fetch_bytes_per_step(1000, hop_count=13)/(1024**3):.2f} GB")
    print(f"Relay bytes:{relay_bytes_per_step(100, 1000)/1024:.2f} kB = {relay_bytes_per_step(100, 1000)/(1024**2):.2f} MB")
