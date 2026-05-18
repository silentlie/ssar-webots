"""
Sensor-driven maze exploration for the Pioneer 3-AT.

Builds a local occupancy grid from sonar and camera readings (no maze file).
Uses frontier-based search on discovered free cells, then grid-aligned motion.
"""

from __future__ import annotations

import math
import struct
import zlib
from collections import deque
from enum import IntEnum
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple, cast

from controller import Camera, DistanceSensor, PositionSensor, Robot

from utils import Wheels

WORLDS_DIR = Path(__file__).resolve().parent.parent.parent / "worlds"
DISCOVERED_MAZE_FILE = WORLDS_DIR / "discovered_maze.txt"
DISCOVERED_MAZE_IMAGE = WORLDS_DIR / "discovered_maze.png"

TILE_SIZE = 1.0
WHEEL_RADIUS = 0.0963
WHEEL_BASE = 0.371

SONAR_COUNT = 16
SONAR_NAMES = [f"so{i}" for i in range(SONAR_COUNT)]

# Sonar mounting positions on the Pioneer 3-AT (robot +X forward, +Y left).
SONAR_XY = (
    (0.1466, 0.1354),
    (0.1945, 0.118),
    (0.2287, 0.077),
    (0.2469, 0.0267),
    (0.247, -0.0267),
    (0.229, -0.077),
    (0.1947, -0.118),
    (0.1469, -0.1354),
    (-0.1447, -0.1354),
    (-0.1925, -0.118),
    (-0.2268, -0.077),
    (-0.2448, -0.0267),
    (-0.2447, 0.0267),
    (-0.2265, 0.077),
    (-0.1922, 0.118),
    (-0.1444, 0.1354),
)
SONAR_ANGLES = tuple(math.atan2(y, x) for x, y in SONAR_XY)

WALL_RANGE = 0.42
OPEN_RANGE = 0.55
SONAR_MAX = 4.5

DRIVE_SPEED = 2.2
TURN_SPEED = 1.8
TURN_TOLERANCE = 0.12
DRIVE_TOLERANCE = 0.08

GridPos = Tuple[int, int]
DIRS: Tuple[GridPos, ...] = ((-1, 0), (0, 1), (1, 0), (0, -1))
DIR_TO_FACING = {d: i for i, d in enumerate(DIRS)}
FACINGS = ("N", "E", "S", "W")
# N, E, S, W relative to robot +X forward.
REL_ANGLES = (math.pi / 2, 0.0, -math.pi / 2, math.pi)


class Cell(IntEnum):
    UNKNOWN = 0
    FREE = 1
    WALL = 2
    GOAL = 3
    DANGER = 4


def _write_png(path: Path, width: int, height: int, rgba_rows: List[bytes]) -> None:
    def chunk(tag: bytes, data: bytes) -> bytes:
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)

    raw = b"".join(b"\x00" + row for row in rgba_rows)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 6, 0, 0, 0)
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 9))
        + chunk(b"IEND", b"")
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


class ExplorationGrid:
    """Sparse grid discovered relative to the robot start cell (0, 0)."""

    def __init__(self) -> None:
        self.cells: Dict[GridPos, Cell] = {(0, 0): Cell.FREE}

    def get(self, pos: GridPos) -> Cell:
        return self.cells.get(pos, Cell.UNKNOWN)

    def set(self, pos: GridPos, value: Cell) -> None:
        current = self.cells.get(pos, Cell.UNKNOWN)
        if value == Cell.UNKNOWN:
            return
        if current == Cell.WALL and value == Cell.FREE:
            return
        if current in (Cell.GOAL, Cell.DANGER) and value == Cell.FREE:
            return
        self.cells[pos] = value

    def is_passable(self, pos: GridPos) -> bool:
        return self.get(pos) in (Cell.FREE, Cell.GOAL)

    def is_frontier(self, pos: GridPos) -> bool:
        if self.get(pos) != Cell.FREE:
            return False
        for n in self.neighbors(pos):
            if self.get(n) == Cell.UNKNOWN:
                return True
        return False

    @staticmethod
    def neighbors(pos: GridPos) -> List[GridPos]:
        r, c = pos
        return [(r + dr, c + dc) for dr, dc in DIRS]

    def bounds(self) -> Tuple[int, int, int, int]:
        rows = [p[0] for p in self.cells]
        cols = [p[1] for p in self.cells]
        return min(rows), max(rows), min(cols), max(cols)

    def save_text(self, path: Path) -> None:
        r0, r1, c0, c1 = self.bounds()
        symbols = {
            Cell.UNKNOWN: "?",
            Cell.FREE: ".",
            Cell.WALL: "#",
            Cell.GOAL: "G",
            Cell.DANGER: "R",
        }
        lines: List[str] = []
        for r in range(r0, r1 + 1):
            line = []
            for c in range(c0, c1 + 1):
                if (r, c) == (0, 0):
                    line.append("S")
                else:
                    line.append(symbols[self.get((r, c))])
            lines.append("".join(line))
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def save_png(self, path: Path, path_cells: Optional[Set[GridPos]] = None) -> None:
        r0, r1, c0, c1 = self.bounds()
        height = r1 - r0 + 1
        width = c1 - c0 + 1
        cell_px = 40
        border = 2
        img_w = width * cell_px
        img_h = height * cell_px
        pixels = bytearray(img_w * img_h * 4)
        colors = {
            Cell.UNKNOWN: (200, 200, 210, 255),
            Cell.FREE: (237, 240, 252, 255),
            Cell.WALL: (40, 40, 40, 255),
            Cell.GOAL: (0, 171, 28, 255),
            Cell.DANGER: (220, 60, 60, 255),
        }

        def fill_rect(x0: int, y0: int, x1: int, y1: int, rgba: tuple[int, int, int, int]) -> None:
            for y in range(y0, y1):
                for x in range(x0, x1):
                    idx = (y * img_w + x) * 4
                    pixels[idx : idx + 4] = bytes(rgba)

        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                pos = (r, c)
                if pos == (0, 0):
                    color = (255, 0, 0, 255)
                elif path_cells and pos in path_cells:
                    color = (220, 235, 113, 255)
                else:
                    color = colors[self.get(pos)]
                y = (r - r0) * cell_px
                x = (c - c0) * cell_px
                fill_rect(
                    x + border,
                    y + border,
                    x + cell_px - border,
                    y + cell_px - border,
                    color,
                )

        rows = [
            bytes(pixels[y * img_w * 4 : (y + 1) * img_w * 4]) for y in range(img_h)
        ]
        _write_png(path, img_w, img_h, rows)


def _safe_encoder(value: float) -> float:
    if math.isnan(value) or math.isinf(value):
        return 0.0
    return value


class WheelOdometry:
    def __init__(self, robot: Robot, timestep: int) -> None:
        self.left = cast(
            PositionSensor, robot.getDevice("front left wheel sensor")
        )
        self.right = cast(
            PositionSensor, robot.getDevice("front right wheel sensor")
        )
        self.left.enable(timestep)
        self.right.enable(timestep)
        self._l_prev = _safe_encoder(self.left.getValue())
        self._r_prev = _safe_encoder(self.right.getValue())
        self._primed = False
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.grid_facing = 1  # E; updated by motion and bootstrap
        self._l_drive_start: Optional[float] = None
        self._r_drive_start: Optional[float] = None
        self._l_turn_start: Optional[float] = None
        self._r_turn_start: Optional[float] = None

    def reset_encoders(self) -> None:
        self._l_prev = _safe_encoder(self.left.getValue())
        self._r_prev = _safe_encoder(self.right.getValue())
        self._l_drive_start = None
        self._r_drive_start = None
        self._l_turn_start = None
        self._r_turn_start = None

    def set_facing(self, index: int) -> None:
        self.grid_facing = index % 4

    def set_facing_from_yaw(self, yaw: float) -> None:
        if math.isnan(yaw) or math.isinf(yaw):
            return
        angle = (yaw + math.pi * 4) % (2 * math.pi)
        quarter = int(round(angle / (math.pi / 2))) % 4
        mapping = (1, 2, 3, 0)  # E, S, W, N
        self.grid_facing = mapping[quarter]
        self.theta = yaw

    def update(self) -> Tuple[float, float, float]:
        l = _safe_encoder(self.left.getValue())
        r = _safe_encoder(self.right.getValue())

        if not self._primed:
            self._l_prev = l
            self._r_prev = r
            self._primed = True
            return self.x, self.y, self.theta

        dl = (l - self._l_prev) * WHEEL_RADIUS
        dr = (r - self._r_prev) * WHEEL_RADIUS
        self._l_prev = l
        self._r_prev = r

        if math.isnan(dl) or math.isnan(dr):
            return self.x, self.y, self.theta

        # Ignore single-timestep encoder spikes (e.g. first valid read after NaN).
        if abs(dl) > 0.4 or abs(dr) > 0.4:
            return self.x, self.y, self.theta

        d_center = (dl + dr) * 0.5
        d_theta = (dr - dl) / WHEEL_BASE
        self.theta += d_theta
        if math.isnan(self.theta):
            self.theta = 0.0
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)
        return self.x, self.y, self.theta

    def distance_since_reset(self) -> float:
        if self._l_drive_start is None or self._r_drive_start is None:
            return 0.0
        l = self.left.getValue()
        r = self.right.getValue()
        dl = abs(l - self._l_drive_start) * WHEEL_RADIUS
        dr = abs(r - self._r_drive_start) * WHEEL_RADIUS
        return (dl + dr) * 0.5

    def heading_delta_since_reset(self) -> float:
        if self._l_turn_start is None or self._r_turn_start is None:
            return 0.0
        l = self.left.getValue()
        r = self.right.getValue()
        dl = (l - self._l_turn_start) * WHEEL_RADIUS
        dr = (r - self._r_turn_start) * WHEEL_RADIUS
        return (dr - dl) / WHEEL_BASE

    def begin_drive_segment(self) -> None:
        self._l_drive_start = self.left.getValue()
        self._r_drive_start = self.right.getValue()
        self._l_turn_start = None
        self._r_turn_start = None

    def begin_turn_segment(self) -> None:
        self._l_turn_start = self.left.getValue()
        self._r_turn_start = self.right.getValue()
        self._l_drive_start = None
        self._r_drive_start = None

    def grid_cell(self) -> GridPos:
        return (int(round(self.y / TILE_SIZE)), int(round(self.x / TILE_SIZE)))

    def facing_index(self) -> int:
        """Grid-facing used for mapping (N=0, E=1, S=2, W=3)."""
        return self.grid_facing


class SonarMapper:
    def __init__(self, robot: Robot, timestep: int) -> None:
        self.sensors: List[DistanceSensor] = []
        for name in SONAR_NAMES:
            device = cast(DistanceSensor, robot.getDevice(name))
            device.enable(timestep)
            self.sensors.append(device)

    def readings(self) -> List[float]:
        values: List[float] = []
        for sensor in self.sensors:
            value = sensor.getValue()
            if math.isinf(value) or math.isnan(value) or value <= 0:
                values.append(SONAR_MAX)
            else:
                values.append(min(value, SONAR_MAX))
        return values

    def update_grid(
        self,
        grid: ExplorationGrid,
        origin: GridPos,
        facing: int,
        readings: List[float],
    ) -> None:
        for dir_idx, (dr, dc) in enumerate(DIRS):
            rel = (dir_idx - facing) % 4
            local_angle = REL_ANGLES[rel]
            best = SONAR_MAX
            for idx, distance in enumerate(readings):
                diff = abs(self._angle_diff(SONAR_ANGLES[idx], local_angle))
                if diff < math.pi / 6:
                    best = min(best, distance)
            neighbor = (origin[0] + dr, origin[1] + dc)
            if best <= WALL_RANGE:
                grid.set(neighbor, Cell.WALL)
            elif best >= OPEN_RANGE and grid.get(neighbor) == Cell.UNKNOWN:
                grid.set(neighbor, Cell.FREE)

    @staticmethod
    def _angle_diff(a: float, b: float) -> float:
        d = (a - b + math.pi) % (2 * math.pi) - math.pi
        return d


class CameraHints:
    def __init__(self, robot: Robot, timestep: int) -> None:
        self.enabled = False
        try:
            self.camera = cast(Camera, robot.getDevice("camera"))
            self.camera.enable(timestep)
            self.enabled = True
        except Exception:
            self.camera = None

    def scan_ahead(self) -> Optional[Cell]:
        if not self.enabled or self.camera is None:
            return None
        try:
            image = self.camera.getImage()
        except ValueError:
            # Camera buffer not ready until after at least one simulation step.
            return None
        if not image:
            return None
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        x0 = width // 3
        x1 = 2 * width // 3
        y0 = height // 3
        y1 = 2 * height // 3
        green = red = count = 0
        for y in range(y0, y1, 2):
            for x in range(x0, x1, 2):
                offset = (y * width + x) * 4
                r = image[offset]
                g = image[offset + 1]
                b = image[offset + 2]
                count += 1
                if g > 120 and g > r + 30 and g > b + 20:
                    green += 1
                if r > 120 and r > g + 30 and r > b + 20:
                    red += 1
        if count == 0:
            return None
        if green / count > 0.12:
            return Cell.GOAL
        if red / count > 0.12:
            return Cell.DANGER
        return None


def plan_path(grid: ExplorationGrid, start: GridPos, goal: GridPos) -> List[GridPos]:
    if start == goal:
        return []
    queue: deque[GridPos] = deque([start])
    came_from: Dict[GridPos, GridPos] = {}
    while queue:
        current = queue.popleft()
        for nxt in grid.neighbors(current):
            if not grid.is_passable(nxt) or nxt in came_from:
                continue
            came_from[nxt] = current
            if nxt == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(came_from[path[-1]])
                path.reverse()
                return path[1:]
            queue.append(nxt)
    return []


def nearest_frontier(grid: ExplorationGrid, start: GridPos) -> Optional[GridPos]:
    queue: deque[GridPos] = deque([start])
    visited: Set[GridPos] = {start}
    while queue:
        current = queue.popleft()
        if grid.is_frontier(current):
            return current
        for nxt in grid.neighbors(current):
            if nxt in visited or not grid.is_passable(nxt):
                continue
            visited.add(nxt)
            queue.append(nxt)
    return None


def nearest_goal(grid: ExplorationGrid, start: GridPos, skip: Set[GridPos]) -> Optional[GridPos]:
    queue: deque[GridPos] = deque([start])
    visited: Set[GridPos] = {start}
    while queue:
        current = queue.popleft()
        if grid.get(current) == Cell.GOAL and current not in skip:
            return current
        for nxt in grid.neighbors(current):
            if nxt in visited or not grid.is_passable(nxt):
                continue
            visited.add(nxt)
            queue.append(nxt)
    return None


class GridMotion:
    IDLE = "idle"
    TURN = "turn"
    DRIVE = "drive"

    def __init__(self, wheels: Wheels) -> None:
        self.wheels = wheels
        self.state = self.IDLE
        self.target_facing = 0
        self._turns_left = 0
        self._turn_left = True

    @property
    def busy(self) -> bool:
        return self.state != self.IDLE

    def start_step(self, from_cell: GridPos, to_cell: GridPos, odom: WheelOdometry) -> None:
        dr = to_cell[0] - from_cell[0]
        dc = to_cell[1] - from_cell[1]
        delta = (dr, dc)
        if delta not in DIR_TO_FACING:
            self.state = self.IDLE
            return
        self.target_facing = DIR_TO_FACING[delta]
        current = odom.facing_index()
        diff = (self.target_facing - current) % 4
        if diff == 0:
            self._start_drive(odom)
        elif diff == 1:
            self._turns_left = 1
            self._turn_left = True
            self._begin_turn(odom)
        elif diff == 3:
            self._turns_left = 1
            self._turn_left = False
            self._begin_turn(odom)
        else:
            self._turns_left = 2
            self._turn_left = True
            self._begin_turn(odom)

    def _start_drive(self, odom: WheelOdometry) -> None:
        self.state = self.DRIVE
        odom.set_facing(self.target_facing)
        odom.begin_drive_segment()
        self.wheels.forward(DRIVE_SPEED)

    def _begin_turn(self, odom: WheelOdometry) -> None:
        self.state = self.TURN
        odom.begin_turn_segment()
        if self._turn_left:
            self.wheels.turn_left(TURN_SPEED)
        else:
            self.wheels.turn_right(TURN_SPEED)

    def step(self, odom: WheelOdometry) -> bool:
        """Advance motion. Returns True when the grid step finished."""
        if self.state == self.TURN:
            turned = abs(odom.heading_delta_since_reset())
            if math.isnan(turned):
                turned = 0.0
            if turned >= math.pi / 2 - TURN_TOLERANCE:
                self.wheels.stop()
                odom.reset_encoders()
                step = 1 if self._turn_left else -1
                odom.set_facing((odom.facing_index() + step) % 4)
                self._turns_left -= 1
                if self._turns_left > 0:
                    self._begin_turn(odom)
                    return False
                odom.set_facing(self.target_facing)
                self._start_drive(odom)
            return False

        if self.state == self.DRIVE:
            if odom.distance_since_reset() >= TILE_SIZE - DRIVE_TOLERANCE:
                self.wheels.stop()
                self.state = self.IDLE
                odom.reset_encoders()
                return True
            return False

        return False


class ExplorationStrategy:
    """
    Online maze exploration: map with sonar, pick frontiers, drive on a grid.
    Does not read worlds/current_maze.txt or any prior map.
    """

    def __init__(self, robot: Robot, wheels: Wheels, timestep: int) -> None:
        self.robot = robot
        self.wheels = wheels
        self.timestep = timestep
        self.grid = ExplorationGrid()
        self.odom = WheelOdometry(robot, timestep)
        self.sonar = SonarMapper(robot, timestep)
        self.camera = CameraHints(robot, timestep)
        self.motion = GridMotion(wheels)
        self.logical_pos: GridPos = (0, 0)
        self.planned_path: List[GridPos] = []
        self.goals_visited: Set[GridPos] = set()
        self.finished = False
        self._steps = 0
        self._status_counter = 0
        self._heading_bootstrapped = False
        print("ExplorationStrategy: mapping maze from sonar (no prior map).")

    def _bootstrap_heading(self) -> None:
        if self._heading_bootstrapped:
            return
        self._heading_bootstrapped = True
        try:
            rot = self.robot.getSelf().getOrientation()
            yaw = math.atan2(rot[3], rot[0])
            self.odom.set_facing_from_yaw(yaw)
        except Exception:
            pass

    def update(self) -> None:
        if self.finished:
            self.wheels.stop()
            return

        self._bootstrap_heading()
        self.odom.update()
        readings = self.sonar.readings()
        facing = self.odom.facing_index()
        self.grid.set(self.logical_pos, Cell.FREE)
        self.sonar.update_grid(self.grid, self.logical_pos, facing, readings)
        self._apply_camera_hint(facing)

        if self.logical_pos in self.goals_visited:
            pass
        elif self.grid.get(self.logical_pos) == Cell.GOAL:
            self.goals_visited.add(self.logical_pos)
            print(f"Reached goal cell {self.logical_pos}")

        if self.motion.busy:
            if self.motion.step(self.odom):
                if self.planned_path:
                    self.logical_pos = self.planned_path.pop(0)
            return

        if self.planned_path:
            nxt = self.planned_path[0]
            self.motion.start_step(self.logical_pos, nxt, self.odom)
            return

        target = nearest_goal(self.grid, self.logical_pos, self.goals_visited)
        if target is None:
            target = nearest_frontier(self.grid, self.logical_pos)

        if target is None:
            self._complete()
            return

        route = plan_path(self.grid, self.logical_pos, target)
        if not route:
            self.grid.set(target, Cell.WALL)
            return

        self.planned_path = route
        self.motion.start_step(self.logical_pos, self.planned_path[0], self.odom)

        self._steps += 1
        if self._steps % 40 == 0:
            self._log_status()

    def _apply_camera_hint(self, facing: int) -> None:
        hint = self.camera.scan_ahead()
        if hint is None:
            return
        dr, dc = DIRS[facing % 4]
        ahead = (self.logical_pos[0] + dr, self.logical_pos[1] + dc)
        self.grid.set(ahead, hint)
        if hint == Cell.GOAL:
            print(f"Camera: goal detected ahead at {ahead}")

    def _log_status(self) -> None:
        known = sum(1 for v in self.grid.cells.values() if v != Cell.UNKNOWN)
        goals = sum(1 for v in self.grid.cells.values() if v == Cell.GOAL)
        print(
            f"Exploring cell {self.logical_pos} | "
            f"known={known} goals_found={goals} goals_visited={len(self.goals_visited)}"
        )

    def _complete(self) -> None:
        self.finished = True
        self.wheels.stop()
        self.grid.save_text(DISCOVERED_MAZE_FILE)
        self.grid.save_png(DISCOVERED_MAZE_IMAGE)
        print("Exploration complete.")
        print(f"  Goals visited: {len(self.goals_visited)}")
        print(f"  Map saved to {DISCOVERED_MAZE_FILE}")
        print(f"  Image saved to {DISCOVERED_MAZE_IMAGE}")
