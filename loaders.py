from dataclasses import dataclass, field
from typing import List, Optional

from vec2 import Vec2

from config import (
    FIELD_SIZE_PX,
    LOADER_RADIUS_PX,
    LOADER_CAPACITY,
)


@dataclass
class Loader:
    name: str
    pos: Vec2
    queue: List[str] = field(default_factory=list)
    wall_shapes: List[object] = field(default_factory=list)
    ball_shapes: List[object] = field(default_factory=list)

    def has_ball(self) -> bool:
        return len(self.queue) > 0

    def pop_ball(self) -> Optional[str]:
        if not self.queue:
            return None
        return self.queue.pop(0)

    def contains_point(self, p: Vec2) -> bool:
        rel = p - self.pos
        if rel.length <= (LOADER_RADIUS_PX + 4.0):
            return True

        inward = Vec2(0.0, 1.0) if self.pos.y < (FIELD_SIZE_PX * 0.5) else Vec2(0.0, -1.0)
        axial = rel.dot(inward)
        side = abs(rel.x)
        return (
            0.0 <= axial <= (LOADER_RADIUS_PX + 34.0)
            and side <= (LOADER_RADIUS_PX * 0.95)
        )


class LoaderManager:
    def __init__(self):
        self.loaders: List[Loader] = []

    def build(self, long_goals):
        self.loaders.clear()

        left_x = float(long_goals[0]["cx"])
        right_x = float(long_goals[1]["cx"])

        y_top = LOADER_RADIUS_PX + 8.0
        y_bot = FIELD_SIZE_PX - (LOADER_RADIUS_PX + 8.0)

        def make_queue(first3: str, last3: str):
            return [first3] * 3 + [last3] * 3

        self.loaders.append(Loader(
            name="TL",
            pos=Vec2(left_x, y_top),
            queue=make_queue("blue", "red"),
        ))
        self.loaders.append(Loader(
            name="TR",
            pos=Vec2(right_x, y_top),
            queue=make_queue("red", "blue"),
        ))
        self.loaders.append(Loader(
            name="BL",
            pos=Vec2(left_x, y_bot),
            queue=make_queue("red", "blue"),
        ))
        self.loaders.append(Loader(
            name="BR",
            pos=Vec2(right_x, y_bot),
            queue=make_queue("blue", "red"),
        ))

    def build_physics(self, space):
        for loader in self.loaders:
            self._build_loader_walls(space, loader)
            self._sync_loader_ball_colliders(space, loader)

    def _build_loader_walls(self, space, loader: Loader):
        radius = LOADER_RADIUS_PX
        wall_t = max(5.0, radius * 0.22)
        wall_len = radius * 2.00
        wall_h_m = 0.52
        inward = Vec2(0.0, 1.0) if loader.pos.y < (FIELD_SIZE_PX * 0.5) else Vec2(0.0, -1.0)
        right = Vec2(1.0, 0.0)

        specs = [
            (loader.pos + right * (radius * 0.92), (wall_t, wall_len), 0.0),
            (loader.pos - right * (radius * 0.92), (wall_t, wall_len), 0.0),
            (loader.pos - inward * (radius * 0.92), (radius * 2.05, wall_t), 0.0),
        ]
        for center, size, angle in specs:
            shape = space.create_static_box(
                (center.x, center.y),
                size,
                angle=angle,
                friction=0.8,
                restitution=0.02,
                height_m=wall_h_m,
                z_center_m=wall_h_m * 0.5,
                visual_rgba=None,
            )
            shape.is_loader_solid = True
            shape.loader_name = loader.name
            loader.wall_shapes.append(shape)

    def _sync_loader_ball_colliders(self, space, loader: Loader):
        if loader.ball_shapes:
            space.remove(*loader.ball_shapes)
            loader.ball_shapes.clear()

        ball_radius = LOADER_RADIUS_PX * 0.62
        bottom_z_m = 0.070
        step_z_m = 0.075
        for slot_index, _color in enumerate(loader.queue[:LOADER_CAPACITY]):
            shape = space.create_static_sphere(
                (loader.pos.x, loader.pos.y),
                ball_radius,
                z_center_m=bottom_z_m + (slot_index * step_z_m),
                friction=0.7,
                restitution=0.02,
                visual_rgba=None,
            )
            shape.is_loader_solid = True
            shape.is_loader_ball = True
            shape.loader_name = loader.name
            loader.ball_shapes.append(shape)

    def try_load_into_robot(self, robot, space=None):
        probe = None
        if hasattr(robot, "loader_paddle_loading_probe"):
            probe = robot.loader_paddle_loading_probe()
        if probe is None:
            return
        if not robot.can_accept_block():
            return

        target = None
        for loader in self.loaders:
            if loader.contains_point(probe):
                target = loader
                break
        if target is None or not target.has_ball():
            return

        color = target.pop_ball()
        if color is None:
            return

        ok = robot.start_pickup_color(color)
        if not ok:
            target.queue.insert(0, color)
        if space is not None:
            self._sync_loader_ball_colliders(space, target)
