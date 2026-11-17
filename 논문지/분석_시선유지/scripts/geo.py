"""ros2/src/rd_bridge/rd_bridge/mav.py 의 좌표 계산을 그대로 옮긴 것.

제어기가 쓴 식과 **같은 식**이어야 e_psi 를 재현할 수 있으므로 복사해 둔다
(원본: mav.py meters_between / bearing_between / offset_to_gps / relative_bearing).
"""
import math

R = 6371000.0


def meters_between(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def bearing_between(lat1, lon1, lat2, lon2):
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return math.degrees(math.atan2(y, x)) % 360.0


def offset_to_gps(lat, lon, heading, fwd, right):
    h = math.radians(heading)
    north = fwd * math.cos(h) - right * math.sin(h)
    east = fwd * math.sin(h) + right * math.cos(h)
    return (lat + math.degrees(north / R),
            lon + math.degrees(east / (R * math.cos(math.radians(lat)))))


def relative_bearing(move_bearing, heading):
    """방위각 → 기수 기준 상대각(-180~180). 0=앞, +90=오른쪽."""
    return (move_bearing - heading + 540.0) % 360.0 - 180.0
