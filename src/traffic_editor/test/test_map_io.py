import tempfile
from pathlib import Path

from traffic_editor.map_io import dump_traffic_map, load_traffic_map


class _Pos:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.x = x
        self.y = y
        self.z = z


class _Pose:
    def __init__(self, x: float, y: float, z: float = 0.0):
        self.position = _Pos(x, y, z)


def test_dump_filters_invalid_connections():
    waypoints = {
        "wp_001": {
            "pose": _Pose(1.0, 2.0),
            "name": "A",
            "connections": ["wp_001", "wp_002", "wp_002", "wp_404"],
            "is_parking_spot": False,
            "is_charging_station": False,
            "radius": 0.5,
        },
        "wp_002": {
            "pose": _Pose(3.0, 4.0),
            "name": "B",
            "connections": [],
            "is_parking_spot": True,
            "is_charging_station": False,
            "radius": 0.6,
        },
    }

    with tempfile.TemporaryDirectory() as td:
        path = str(Path(td) / "traffic.yaml")
        dump_traffic_map(path, waypoints, map_yaml_path="", map_image_file="")
        loaded_waypoints, _, _ = load_traffic_map(path)

    # Self-link and missing waypoint should be filtered; duplicate should dedup.
    assert loaded_waypoints["wp_001"]["connections"] == ["wp_002"]


def test_load_enforces_undirected_connections_and_counter():
    content = """
map_id: map0
map_name: test
waypoints:
  - id: wp_001
    name: A
    position: {x: 1.0, y: 2.0, z: 0.0}
    connections: [wp_002]
    is_parking_spot: false
    is_charging_station: false
    radius: 0.5
  - id: wp_002
    name: B
    position: {x: 3.0, y: 4.0, z: 0.0}
    connections: []
    is_parking_spot: false
    is_charging_station: false
    radius: 0.5
"""
    with tempfile.TemporaryDirectory() as td:
        path = Path(td) / "map.yaml"
        path.write_text(content, encoding="utf-8")
        waypoints, connections, max_num = load_traffic_map(str(path))

    assert "wp_001" in waypoints["wp_002"]["connections"]
    assert ("wp_002", "wp_001") in connections
    assert max_num == 2
