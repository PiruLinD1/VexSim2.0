import math
import random
from dataclasses import dataclass, field
from typing import List, Optional

import pygame

from vec2 import Vec2

from config import (
    FIELD_SIZE_PX,
    PPM,
    ROBOT_SIZE_PX,
    BLOCK_RADIUS_PX,
    LONG_GOAL_CAPACITY,
    MIDDLE_GOAL_CAPACITY,
    DUMP_FLOOR_SPEED,
    clamp,
    inches_to_m,
    inches_to_px,
    newton_to_sim_force,
    px_to_meters,
    pxps_to_mps,
)
from physics import (
    BLOCK_BODY_HEIGHT_M,
    BLOCK_COLLISION_GROUP,
    BLOCK_CONTACT_FRICTION,
    BLOCK_CONTACT_RESTITUTION,
    BLOCK_CONTACT_ROLLING_FRICTION,
    BLOCK_CONTACT_SPINNING_FRICTION,
    bullet,
    spawn_block,
)


LONG_GOAL_LENGTH_IN = 55.5
LONG_GOAL_CONTROL_LENGTH_IN = 13.33
MIDDLE_GOAL_LENGTH_IN = 30.0

GOAL_INNER_DIAMETER_IN = 4.10
GOAL_WALL_THICKNESS_IN = 0.18
GOAL_WALL_HEIGHT_IN = 4.00
GOAL_FLOOR_THICKNESS_IN = 0.22
LONG_GOAL_ENTRY_GUIDE_LENGTH_IN = 1.20
MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_IN = 0.00

LONG_GOAL_FLOOR_TOP_M = 0.40
LOW_MIDDLE_GOAL_FLOOR_TOP_M = inches_to_m(GOAL_FLOOR_THICKNESS_IN)
HIGH_MIDDLE_GOAL_FLOOR_TOP_M = 0.20
LONG_GOAL_SUPPORT_TOP_GAP_M = 0.006
LONG_GOAL_UNDER_BLOCKER_BOTTOM_M = 0.15
LONG_GOAL_UNDER_BLOCKER_HEIGHT_M = 0.10
LONG_GOAL_UNDER_BLOCKER_WIDTH_SCALE = 0.82
LONG_GOAL_UNDER_BLOCKER_LENGTH_MARGIN_PX = BLOCK_RADIUS_PX * 0.80

GOAL_SURFACE_FRICTION = 2.64
GOAL_SURFACE_RESTITUTION = 0.06
GOAL_SURFACE_ROLLING_FRICTION = 0.0360
GOAL_SURFACE_SPINNING_FRICTION = 0.0456

GOAL_BLOCK_CONTACT_FRICTION = 0.025
GOAL_BLOCK_CONTACT_RESTITUTION = 0.96
GOAL_BLOCK_CONTACT_ROLLING_FRICTION = 0.00008
GOAL_BLOCK_CONTACT_SPINNING_FRICTION = 0.00012
GOAL_BLOCK_LINEAR_DAMPING = 0.0035
GOAL_BLOCK_ANGULAR_DAMPING = 0.022

LONG_GOAL_ENTRY_SPEED_GAIN = 0.34
MIDDLE_GOAL_ENTRY_SPEED_GAIN = 0.92
GOAL_ENTRY_SPEED_JITTER = 0.02
GOAL_ENTRY_SIDE_SPEED_GAIN = 0.004
GOAL_ENTRY_DEPTH_IN = 1.20
GOAL_ENTRY_LATERAL_JITTER_IN = 0.0
GOAL_ENTRY_SPIN_RAD_S = 0.35
GOAL_ENTRY_EMPTY_BOOST_EXP = 2.0
LONG_GOAL_ENTRY_EMPTY_SPEED_BOOST = 0.55
MIDDLE_GOAL_ENTRY_EMPTY_SPEED_BOOST = 0.70
LONG_GOAL_ENTRY_EMPTY_PUSH_BOOST = 1.10
MIDDLE_GOAL_ENTRY_EMPTY_PUSH_BOOST = 1.35

GOAL_ENTRY_PUSH_TIME_SEC = 0.06
LONG_GOAL_ENTRY_PUSH_FORCE_N = 0.26
MIDDLE_GOAL_ENTRY_PUSH_FORCE_N = 0.42
GOAL_ENTRY_PUSH_REACH_PX = BLOCK_RADIUS_PX * 2.8
GOAL_ENTRY_PUSH_OUTSIDE_PX = BLOCK_RADIUS_PX * 0.80
GOAL_CHAIN_PUSH_GAIN = 0.0
GOAL_CHAIN_PUSH_DECAY = 1.0
GOAL_PACKING_HOLD_SEC = 0.18
LONG_GOAL_PACKING_ORIGIN_PX = BLOCK_RADIUS_PX * 1.35
MIDDLE_GOAL_PACKING_ORIGIN_PX = BLOCK_RADIUS_PX * 1.25
LONG_GOAL_EMPTY_PACK_ORIGIN_BONUS_PX = BLOCK_RADIUS_PX * 5.20
MIDDLE_GOAL_EMPTY_PACK_ORIGIN_BONUS_PX = BLOCK_RADIUS_PX * 2.60
GOAL_PACKING_TAIL_MARGIN_PX = BLOCK_RADIUS_PX * 0.80
LONG_GOAL_PACKING_FILL_RATIO = 1.03
MIDDLE_GOAL_PACKING_FILL_RATIO = 1.12
GOAL_PACKING_FORCE_MAX_N = 0.14
GOAL_PACKING_RETURN_FORCE_MAX_N = 0.05
GOAL_PACKING_STIFFNESS_N_PER_M = 3.0
GOAL_PACKING_DAMP_N_PER_MPS = 0.28
MIDDLE_GOAL_PACKING_FORCE_SCALE = 0.72
GOAL_OVERFLOW_RESIST_FORCE_N = 0.92
GOAL_OVERFLOW_RESIST_REACH_PX = BLOCK_RADIUS_PX * 2.6
GOAL_OVERFLOW_REVERSE_DAMP = 0.55
LONG_GOAL_OVERFLOW_EJECT_FORCE_N = 0.76
LONG_GOAL_OVERFLOW_EJECT_FORCE_MAX_N = 1.10
LONG_GOAL_OVERFLOW_MOUTH_PUSH_BOOST = 1.18
LONG_GOAL_CHAIN_PUSH_BOOST = 1.18
MIDDLE_GOAL_OVERFLOW_EJECT_FORCE_N = 1.08
MIDDLE_GOAL_OVERFLOW_EJECT_FORCE_MAX_N = 1.55
MIDDLE_GOAL_OVERFLOW_MOUTH_PUSH_BOOST = 1.14
MIDDLE_GOAL_CHAIN_PUSH_BOOST = 1.14
GOAL_CENTERING_STIFFNESS_N_PER_M = 2.8
GOAL_CENTERING_DAMP_N_PER_MPS = 0.22
GOAL_CENTERING_FORCE_MAX_N = 0.08
GOAL_AXIAL_DAMP_N_PER_MPS = 0.0
GOAL_AXIAL_DAMP_FORCE_MAX_N = 0.0
MIDDLE_GOAL_CENTERING_FORCE_SCALE = 0.26
MIDDLE_GOAL_AXIAL_DAMP_SCALE = 0.0
GOAL_MOUTH_BACKSTOP_REACH_PX = BLOCK_RADIUS_PX * 2.4
GOAL_MOUTH_BACKSTOP_FORCE_N = 0.26
GOAL_MOUTH_BACKSTOP_FORCE_MAX_N = 0.48
GOAL_MOUTH_REVERSE_VEL_DAMP = 0.36
LONG_GOAL_TAIL_BACKSTOP_REACH_PX = BLOCK_RADIUS_PX * 5.0
LONG_GOAL_TAIL_BACKSTOP_FORCE_N = 1.20
LONG_GOAL_TAIL_BACKSTOP_FORCE_MAX_N = 2.20
LONG_GOAL_TAIL_REVERSE_VEL_DAMP = 0.72
MIDDLE_GOAL_TAIL_BACKSTOP_REACH_PX = BLOCK_RADIUS_PX * 4.0
MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_N = 4.20
MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_MAX_N = 8.60
MIDDLE_GOAL_TAIL_REVERSE_VEL_DAMP = 1.65
GOAL_TRACK_EDGE_MARGIN_PX = BLOCK_RADIUS_PX * 0.55
GOAL_TRACK_SIDE_TOLERANCE_PX = BLOCK_RADIUS_PX * 0.70
GOAL_TRACK_Z_MARGIN_M = max(0.010, BLOCK_BODY_HEIGHT_M * 0.35)
GOAL_SCORE_Z_MARGIN_M = max(0.006, BLOCK_BODY_HEIGHT_M * 0.25)
LONG_GOAL_DUMP_ZONE_OUTSIDE_MARGIN_PX = BLOCK_RADIUS_PX * 0.90
MIDDLE_GOAL_DUMP_ZONE_OUTSIDE_MARGIN_PX = BLOCK_RADIUS_PX * 0.45
LONG_GOAL_ZONE_MOUTH_OVERLAP_PX = BLOCK_RADIUS_PX * 2.00

GOAL_IMPACT_MIN_NORMAL_FORCE_N = 18.0
GOAL_IMPACT_MIN_CLOSING_SPEED_MPS = 0.03
GOAL_IMPACT_SHAKE_SCALE = 0.50
GOAL_IMPACT_FORCE_SCALE = 0.0024
GOAL_IMPACT_SPEED_FORCE_SCALE = 1.30
GOAL_IMPACT_FORCE_MAX_N = 2.40
GOAL_IMPACT_AXIS_SHAKE_BLEND = 0.92
GOAL_IMPACT_LATERAL_SHAKE_BLEND = 0.31
GOAL_IMPACT_LATERAL_JITTER_RATIO = 0.17
GOAL_IMPACT_DEPTH_DECAY = 0.87
GOAL_IMPACT_END_EJECT_RATIO = 0.23
GOAL_IMPACT_Z_KICK_RATIO = 0.28
GOAL_IMPACT_Z_KICK_MAX_N = 0.63
GOAL_IMPACT_COOLDOWN_SEC = 0.06
GOAL_IMPACT_RELAX_SEC = 0.26

GOAL_ENTRY_CLEARANCE_PX = BLOCK_RADIUS_PX * 1.70
GOAL_ENTRY_DEPTH_STEP_PX = BLOCK_RADIUS_PX * 0.90
GOAL_SCORE_EDGE_MARGIN_PX = BLOCK_RADIUS_PX * 0.03
GOAL_SCORE_SIDE_TOLERANCE_PX = BLOCK_RADIUS_PX * 0.20
GOAL_MOUTH_GATE_COLLISION_GROUP = 32
GOAL_MOUTH_GATE_ACTIVE_SEC = 0.42
GOAL_MOUTH_GATE_WIDTH_SCALE = 0.92
GOAL_MOUTH_GATE_THICKNESS_PX = BLOCK_RADIUS_PX * 0.72
GOAL_MOUTH_GATE_OUTSIDE_OFFSET_PX = BLOCK_RADIUS_PX * 0.42
GOAL_MOUTH_GATE_HEIGHT_M = BLOCK_BODY_HEIGHT_M * 1.08
GOAL_MOUTH_GATE_FRICTION = 0.52
GOAL_MOUTH_GATE_RESTITUTION = 0.02

GOAL_INNER_WIDTH_PX = inches_to_px(GOAL_INNER_DIAMETER_IN)
GOAL_WALL_THICKNESS_PX = inches_to_px(GOAL_WALL_THICKNESS_IN)
GOAL_OUTER_WIDTH_PX = GOAL_INNER_WIDTH_PX + (2.0 * GOAL_WALL_THICKNESS_PX)
GOAL_ENTRY_DEPTH_PX = inches_to_px(GOAL_ENTRY_DEPTH_IN)
GOAL_ENTRY_LATERAL_JITTER_PX = inches_to_px(GOAL_ENTRY_LATERAL_JITTER_IN)
LONG_GOAL_ENTRY_GUIDE_LENGTH_PX = inches_to_px(LONG_GOAL_ENTRY_GUIDE_LENGTH_IN)
MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_PX = inches_to_px(MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_IN)

GOAL_WALL_HEIGHT_M = inches_to_m(GOAL_WALL_HEIGHT_IN)
GOAL_FLOOR_THICKNESS_M = inches_to_m(GOAL_FLOOR_THICKNESS_IN)


@dataclass
class GoalPush:
    mouth_center: Vec2
    inward: Vec2
    time_left: float
    force_n: float


@dataclass
class GoalMouthGate:
    shape: object
    time_left: float = 0.0


@dataclass
class DumpZone:
    rect: pygame.Rect
    goal_kind: str
    goal_index: int
    end: str


@dataclass
class GoalBall:
    shape: object
    color: str
    s: float


@dataclass
class LongGoal:
    cx: float
    top_y: float
    bot_y: float
    capacity: int
    center: Vec2
    axis: Vec2
    normal: Vec2
    half_len: float
    inner_width: float
    floor_top_m: float
    block_center_z_m: float
    entry_guide_length_px: float
    entry_speed_gain: float
    entry_push_force_n: float
    queue: List[GoalBall] = field(default_factory=list)
    tracked_shapes: List[object] = field(default_factory=list)
    wall_shapes: List[object] = field(default_factory=list)
    contact_body_ids: set[int] = field(default_factory=set)
    active_pushes: List[GoalPush] = field(default_factory=list)
    pack_mouth_center: Optional[Vec2] = None
    pack_inward: Optional[Vec2] = None
    pack_time_left: float = 0.0
    control_start_s: float = 0.0
    control_end_s: float = 1.0


@dataclass
class MiddleGoal:
    center: Vec2
    axis: Vec2
    normal: Vec2
    half_len: float
    capacity: int
    inner_width: float
    floor_top_m: float
    block_center_z_m: float
    entry_guide_length_px: float
    entry_speed_gain: float
    entry_push_force_n: float
    queue: List[GoalBall] = field(default_factory=list)
    tracked_shapes: List[object] = field(default_factory=list)
    wall_shapes: List[object] = field(default_factory=list)
    contact_body_ids: set[int] = field(default_factory=set)
    active_pushes: List[GoalPush] = field(default_factory=list)
    pack_mouth_center: Optional[Vec2] = None
    pack_inward: Optional[Vec2] = None
    pack_time_left: float = 0.0


class GoalManager:
    def __init__(self):
        self.dump_zones: List[DumpZone] = []
        self.long_goals: List[LongGoal] = []
        self.middle_goals: List[MiddleGoal] = []
        self.long_shapes: List[object] = []
        self.middle_shapes: List[object] = []
        self.field_floor_body_id: Optional[int] = None
        self._goal_by_contact_body_id: dict[int, object] = {}
        self._goal_impact_cooldowns: dict[int, float] = {}
        self._goal_impact_relax_timers: dict[int, float] = {}
        self._mouth_gates: dict[tuple[str, int, str], GoalMouthGate] = {}

    def build(self, space):
        self.dump_zones.clear()
        self.long_goals.clear()
        self.middle_goals.clear()
        self.long_shapes.clear()
        self.middle_shapes.clear()
        self.field_floor_body_id = self._find_field_floor_body_id(space)
        self._goal_by_contact_body_id.clear()
        self._goal_impact_cooldowns.clear()
        self._goal_impact_relax_timers.clear()
        self._mouth_gates.clear()
        self.long_shapes = self._add_long_goals(space)
        self.middle_shapes = self._add_middle_goals(space)

    def _find_field_floor_body_id(self, space) -> Optional[int]:
        for shape in getattr(space, "shapes", []):
            if getattr(shape, "is_field_floor", False):
                return shape.body.body_id

        for shape in getattr(space, "shapes", []):
            if (not shape.body.dynamic) and (shape.body.z < 0.0):
                return shape.body.body_id
        return None

    def _tube_angle(self, axis: Vec2) -> float:
        return axis.angle + (math.pi * 0.5)

    def _zone_center_outside_goal(
        self,
        center: Vec2,
        axis: Vec2,
        half_len: float,
        sign: float,
        zone_size: float,
        guide_length_px: float,
        outside_margin_px: float,
    ) -> Vec2:
        outward_dist = half_len + guide_length_px + outside_margin_px + (zone_size * 0.5)
        return center + (axis * (sign * outward_dist))

    def _long_zone_center_at_mouth(self, goal, sign: float, zone_size: float) -> Vec2:
        mouth_center = goal.center + (goal.axis * (sign * goal.half_len))
        outward = sign * max(0.0, (zone_size * 0.5) - LONG_GOAL_ZONE_MOUTH_OVERLAP_PX)
        return mouth_center + (goal.axis * outward)

    def _create_tube_shell(
        self,
        space,
        center: Vec2,
        axis: Vec2,
        length_px: float,
        floor_top_m: float,
        guide_length_px: float,
    ):
        normal = axis.perpendicular().normalized()
        angle = self._tube_angle(axis)
        shapes = []
        half_len = length_px * 0.5
        floor_center_z_m = floor_top_m - (GOAL_FLOOR_THICKNESS_M * 0.5)
        wall_center_z_m = floor_top_m + (GOAL_WALL_HEIGHT_M * 0.5)

        floor_shape = space.create_static_box(
            (center.x, center.y),
            (GOAL_INNER_WIDTH_PX, length_px),
            angle=angle,
            friction=GOAL_SURFACE_FRICTION,
            restitution=GOAL_SURFACE_RESTITUTION,
            rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
            spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
            height_m=GOAL_FLOOR_THICKNESS_M,
            z_center_m=floor_center_z_m,
        )
        floor_shape.goal_render_role = "floor"
        shapes.append(floor_shape)

        wall_offset = normal * (0.5 * (GOAL_INNER_WIDTH_PX + GOAL_WALL_THICKNESS_PX))
        wall_size = (GOAL_WALL_THICKNESS_PX, length_px)

        left_wall = space.create_static_box(
            (center.x + wall_offset.x, center.y + wall_offset.y),
            wall_size,
            angle=angle,
            friction=GOAL_SURFACE_FRICTION,
            restitution=GOAL_SURFACE_RESTITUTION,
            rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
            spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
            height_m=GOAL_WALL_HEIGHT_M,
            z_center_m=wall_center_z_m,
        )
        left_wall.goal_render_role = "wall"
        shapes.append(left_wall)

        right_wall = space.create_static_box(
            (center.x - wall_offset.x, center.y - wall_offset.y),
            wall_size,
            angle=angle,
            friction=GOAL_SURFACE_FRICTION,
            restitution=GOAL_SURFACE_RESTITUTION,
            rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
            spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
            height_m=GOAL_WALL_HEIGHT_M,
            z_center_m=wall_center_z_m,
        )
        right_wall.goal_render_role = "wall"
        shapes.append(right_wall)

        if guide_length_px > 1e-6:
            guide_floor_size = (GOAL_INNER_WIDTH_PX, guide_length_px)
            guide_wall_size = (GOAL_WALL_THICKNESS_PX, guide_length_px)
            guide_wall_offset = normal * (0.5 * (GOAL_INNER_WIDTH_PX + GOAL_WALL_THICKNESS_PX))

            for end_sign in (-1.0, 1.0):
                guide_center = center + (axis * (end_sign * (half_len + (guide_length_px * 0.5))))

                guide_floor = space.create_static_box(
                    (guide_center.x, guide_center.y),
                    guide_floor_size,
                    angle=angle,
                    friction=GOAL_SURFACE_FRICTION,
                    restitution=GOAL_SURFACE_RESTITUTION,
                    rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
                    spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
                    height_m=GOAL_FLOOR_THICKNESS_M,
                    z_center_m=floor_center_z_m,
                )
                guide_floor.goal_render_role = "guide"
                shapes.append(guide_floor)

                guide_left = space.create_static_box(
                    (guide_center.x + guide_wall_offset.x, guide_center.y + guide_wall_offset.y),
                    guide_wall_size,
                    angle=angle,
                    friction=GOAL_SURFACE_FRICTION,
                    restitution=GOAL_SURFACE_RESTITUTION,
                    rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
                    spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
                    height_m=GOAL_WALL_HEIGHT_M,
                    z_center_m=wall_center_z_m,
                )
                guide_left.goal_render_role = "guide"
                shapes.append(guide_left)

                guide_right = space.create_static_box(
                    (guide_center.x - guide_wall_offset.x, guide_center.y - guide_wall_offset.y),
                    guide_wall_size,
                    angle=angle,
                    friction=GOAL_SURFACE_FRICTION,
                    restitution=GOAL_SURFACE_RESTITUTION,
                    rolling_friction=GOAL_SURFACE_ROLLING_FRICTION,
                    spinning_friction=GOAL_SURFACE_SPINNING_FRICTION,
                    height_m=GOAL_WALL_HEIGHT_M,
                    z_center_m=wall_center_z_m,
                )
                guide_right.goal_render_role = "guide"
                shapes.append(guide_right)

        for shape in shapes:
            shape.is_goal_surface = True

        body_ids = {shape.body.body_id for shape in shapes}
        return shapes, body_ids, normal

    def _create_long_goal_supports(self, space, center: Vec2, length_px: float):
        support_height_m = max(0.04, LONG_GOAL_FLOOR_TOP_M - LONG_GOAL_SUPPORT_TOP_GAP_M)
        leg_size = (max(6.0, GOAL_WALL_THICKNESS_PX * 3.2), max(10.0, BLOCK_RADIUS_PX * 0.9))
        normal_offsets = (-0.42 * GOAL_OUTER_WIDTH_PX, 0.42 * GOAL_OUTER_WIDTH_PX)
        axial_offsets = (
            -(0.5 * length_px - (leg_size[1] * 1.1)),
            (0.5 * length_px - (leg_size[1] * 1.1)),
        )
        supports = []
        for axial in axial_offsets:
            for normal_offset in normal_offsets:
                support = space.create_static_box(
                    (center.x + normal_offset, center.y + axial),
                    leg_size,
                    friction=1.0,
                    restitution=GOAL_SURFACE_RESTITUTION,
                    height_m=support_height_m,
                    z_center_m=support_height_m * 0.5,
                )
                support.goal_render_role = "support"
                support.is_goal_surface = True
                supports.append(support)

        blocker_width = max(BLOCK_RADIUS_PX * 2.0, GOAL_OUTER_WIDTH_PX * LONG_GOAL_UNDER_BLOCKER_WIDTH_SCALE)
        blocker_length = max(
            BLOCK_RADIUS_PX * 6.0,
            length_px - (2.0 * LONG_GOAL_UNDER_BLOCKER_LENGTH_MARGIN_PX),
        )
        under_blocker = space.create_static_box(
            (center.x, center.y),
            (blocker_width, blocker_length),
            friction=1.0,
            restitution=GOAL_SURFACE_RESTITUTION,
            height_m=LONG_GOAL_UNDER_BLOCKER_HEIGHT_M,
            z_center_m=LONG_GOAL_UNDER_BLOCKER_BOTTOM_M + (LONG_GOAL_UNDER_BLOCKER_HEIGHT_M * 0.5),
        )
        under_blocker.goal_render_role = "under_blocker"
        under_blocker.is_goal_surface = True
        supports.append(under_blocker)
        return supports

    def _add_long_goals(self, space) -> List[object]:
        shapes: List[object] = []
        goal_len_px = inches_to_px(LONG_GOAL_LENGTH_IN)
        half_len = goal_len_px * 0.5
        cy = FIELD_SIZE_PX * 0.5

        def add_one_long(cx: float):
            center = Vec2(cx, cy)
            axis = Vec2(0.0, 1.0)
            shell_shapes, body_ids, normal = self._create_tube_shell(
                space,
                center,
                axis,
                goal_len_px,
                floor_top_m=LONG_GOAL_FLOOR_TOP_M,
                guide_length_px=LONG_GOAL_ENTRY_GUIDE_LENGTH_PX,
            )
            support_shapes = self._create_long_goal_supports(space, center, goal_len_px)
            shell_shapes.extend(support_shapes)
            body_ids.update(shape.body.body_id for shape in support_shapes)
            control_half_len = inches_to_px(LONG_GOAL_CONTROL_LENGTH_IN) * 0.5
            control_norm = control_half_len / max(goal_len_px, 1e-6)
            goal = LongGoal(
                cx=cx,
                top_y=cy - half_len,
                bot_y=cy + half_len,
                capacity=LONG_GOAL_CAPACITY,
                center=center,
                axis=axis,
                normal=normal,
                half_len=half_len,
                inner_width=GOAL_INNER_WIDTH_PX,
                floor_top_m=LONG_GOAL_FLOOR_TOP_M,
                block_center_z_m=LONG_GOAL_FLOOR_TOP_M + (BLOCK_BODY_HEIGHT_M * 0.5),
                entry_guide_length_px=LONG_GOAL_ENTRY_GUIDE_LENGTH_PX,
                entry_speed_gain=LONG_GOAL_ENTRY_SPEED_GAIN,
                entry_push_force_n=LONG_GOAL_ENTRY_PUSH_FORCE_N,
                wall_shapes=shell_shapes,
                contact_body_ids=body_ids,
                control_start_s=0.5 - control_norm,
                control_end_s=0.5 + control_norm,
            )
            self._register_goal_contact_bodies(goal)
            return shell_shapes, goal

        left_shapes, left_goal = add_one_long(24.0 * PPM)
        right_shapes, right_goal = add_one_long(FIELD_SIZE_PX - (24.0 * PPM))

        shapes.extend(left_shapes)
        shapes.extend(right_shapes)
        self.long_goals.extend([left_goal, right_goal])

        zone_size = ROBOT_SIZE_PX * 0.75

        def make_zone(center_pos: Vec2, goal_index: int, end: str):
            rect = pygame.Rect(
                int(center_pos.x - zone_size / 2),
                int(center_pos.y - zone_size / 2),
                int(zone_size),
                int(zone_size),
            )
            self.dump_zones.append(DumpZone(rect, "long", goal_index, end))

        make_zone(self._long_zone_center_at_mouth(left_goal, -1.0, zone_size), 0, "top")
        make_zone(self._long_zone_center_at_mouth(left_goal, 1.0, zone_size), 0, "bottom")
        make_zone(self._long_zone_center_at_mouth(right_goal, -1.0, zone_size), 1, "top")
        make_zone(self._long_zone_center_at_mouth(right_goal, 1.0, zone_size), 1, "bottom")
        self._mouth_gates[("long", 0, "top")] = self._create_goal_mouth_gate(space, left_goal, left_goal.axis)
        self._mouth_gates[("long", 0, "bottom")] = self._create_goal_mouth_gate(space, left_goal, -left_goal.axis)
        self._mouth_gates[("long", 1, "top")] = self._create_goal_mouth_gate(space, right_goal, right_goal.axis)
        self._mouth_gates[("long", 1, "bottom")] = self._create_goal_mouth_gate(space, right_goal, -right_goal.axis)

        return shapes

    def _add_middle_goals(self, space) -> List[object]:
        shapes: List[object] = []
        length_px = inches_to_px(MIDDLE_GOAL_LENGTH_IN)
        half_len = length_px * 0.5
        center = Vec2(FIELD_SIZE_PX * 0.5, FIELD_SIZE_PX * 0.5)
        goal_specs = [
            (Vec2(1.0, -1.0).normalized(), LOW_MIDDLE_GOAL_FLOOR_TOP_M, "low"),
            (Vec2(-1.0, -1.0).normalized(), HIGH_MIDDLE_GOAL_FLOOR_TOP_M, "high"),
        ]

        for axis, floor_top_m, height_role in goal_specs:
            shell_shapes, body_ids, normal = self._create_tube_shell(
                space,
                center,
                axis,
                length_px,
                floor_top_m=floor_top_m,
                guide_length_px=MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_PX,
            )
            for shell_shape in shell_shapes:
                shell_shape.goal_height_role = height_role
            goal = MiddleGoal(
                center=center,
                axis=axis,
                normal=normal,
                half_len=half_len,
                capacity=MIDDLE_GOAL_CAPACITY,
                inner_width=GOAL_INNER_WIDTH_PX,
                floor_top_m=floor_top_m,
                block_center_z_m=floor_top_m + (BLOCK_BODY_HEIGHT_M * 0.5),
                entry_guide_length_px=MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_PX,
                entry_speed_gain=MIDDLE_GOAL_ENTRY_SPEED_GAIN,
                entry_push_force_n=MIDDLE_GOAL_ENTRY_PUSH_FORCE_N,
                wall_shapes=shell_shapes,
                contact_body_ids=body_ids,
            )
            self._register_goal_contact_bodies(goal)
            self.middle_goals.append(goal)
            shapes.extend(shell_shapes)

        zone_size = ROBOT_SIZE_PX * 0.70

        def make_zone_for_end(goal_index: int, end_label: str):
            goal = self.middle_goals[goal_index]
            sign = 1.0 if end_label == "A" else -1.0
            pos = self._zone_center_outside_goal(
                goal.center,
                goal.axis,
                goal.half_len,
                sign,
                zone_size,
                MIDDLE_GOAL_ENTRY_GUIDE_LENGTH_PX,
                MIDDLE_GOAL_DUMP_ZONE_OUTSIDE_MARGIN_PX,
            )
            rect = pygame.Rect(int(pos.x - zone_size / 2), int(pos.y - zone_size / 2), int(zone_size), int(zone_size))
            self.dump_zones.append(DumpZone(rect, "middle", goal_index, end_label))

        make_zone_for_end(0, "A")
        make_zone_for_end(0, "B")
        make_zone_for_end(1, "A")
        make_zone_for_end(1, "B")
        self._mouth_gates[("middle", 0, "A")] = self._create_goal_mouth_gate(space, self.middle_goals[0], -self.middle_goals[0].axis)
        self._mouth_gates[("middle", 0, "B")] = self._create_goal_mouth_gate(space, self.middle_goals[0], self.middle_goals[0].axis)
        self._mouth_gates[("middle", 1, "A")] = self._create_goal_mouth_gate(space, self.middle_goals[1], -self.middle_goals[1].axis)
        self._mouth_gates[("middle", 1, "B")] = self._create_goal_mouth_gate(space, self.middle_goals[1], self.middle_goals[1].axis)

        return shapes

    def find_zone_for_points(self, points: List[Vec2]) -> Optional[DumpZone]:
        for zone in self.dump_zones:
            for point in points:
                if zone.rect.collidepoint(point.x, point.y):
                    return zone
        return None

    def _goal_local_coords(self, goal, pos: Vec2):
        rel = pos - goal.center
        axial = rel.dot(goal.axis)
        lateral = rel.dot(goal.normal)
        return axial, lateral

    def _goal_match_limits(self, goal, *, include_guides: bool):
        axial_limit = goal.half_len - GOAL_SCORE_EDGE_MARGIN_PX
        side_limit = (goal.inner_width * 0.5) + GOAL_SCORE_SIDE_TOLERANCE_PX
        z_margin_m = GOAL_SCORE_Z_MARGIN_M
        if include_guides:
            axial_limit = goal.half_len + goal.entry_guide_length_px + GOAL_TRACK_EDGE_MARGIN_PX
            side_limit = (goal.inner_width * 0.5) + GOAL_TRACK_SIDE_TOLERANCE_PX
            z_margin_m = GOAL_TRACK_Z_MARGIN_M
        return axial_limit, side_limit, z_margin_m

    def _shape_inside_goal_volume(self, shape, goal, *, include_guides: bool):
        axial, lateral = self._goal_local_coords(goal, shape.body.position)
        axial_limit, side_limit, z_margin_m = self._goal_match_limits(goal, include_guides=include_guides)
        if abs(axial) > max(0.0, axial_limit):
            return None
        if abs(lateral) > side_limit:
            return None

        z_min = max(0.0, goal.floor_top_m - z_margin_m)
        z_max = goal.floor_top_m + GOAL_WALL_HEIGHT_M + BLOCK_BODY_HEIGHT_M
        if not (z_min <= shape.body.z <= z_max):
            return None

        return axial, lateral

    def _goal_fill_fraction(self, goal) -> float:
        tracked_count = max(len(goal.tracked_shapes), len(goal.queue))
        return clamp(tracked_count / max(1.0, float(goal.capacity)), 0.0, 1.0)

    def _goal_entry_boost(self, goal, *, speed: bool) -> float:
        fill_fraction = self._goal_fill_fraction(goal)
        empty_ratio = (1.0 - fill_fraction) ** GOAL_ENTRY_EMPTY_BOOST_EXP
        if self._goal_is_long(goal):
            bonus = LONG_GOAL_ENTRY_EMPTY_SPEED_BOOST if speed else LONG_GOAL_ENTRY_EMPTY_PUSH_BOOST
        else:
            bonus = MIDDLE_GOAL_ENTRY_EMPTY_SPEED_BOOST if speed else MIDDLE_GOAL_ENTRY_EMPTY_PUSH_BOOST
        return 1.0 + (bonus * empty_ratio)

    def _entry_speed_pxps(self, goal, base_floor_speed_pxps: float | None = None) -> float:
        floor_speed = DUMP_FLOOR_SPEED if base_floor_speed_pxps is None else float(base_floor_speed_pxps)
        base_speed = max(floor_speed * goal.entry_speed_gain, BLOCK_RADIUS_PX * 4.8)
        return base_speed * self._goal_entry_boost(goal, speed=True)

    def _entry_push_force_n(self, goal) -> float:
        return goal.entry_push_force_n * self._goal_entry_boost(goal, speed=False)

    def _goal_is_long(self, goal) -> bool:
        return goal.capacity > MIDDLE_GOAL_CAPACITY

    def _goal_chain_push_boost(self, goal) -> float:
        return LONG_GOAL_CHAIN_PUSH_BOOST if self._goal_is_long(goal) else MIDDLE_GOAL_CHAIN_PUSH_BOOST

    def _goal_overflow_mouth_push_boost(self, goal) -> float:
        return LONG_GOAL_OVERFLOW_MOUTH_PUSH_BOOST if self._goal_is_long(goal) else MIDDLE_GOAL_OVERFLOW_MOUTH_PUSH_BOOST

    def _goal_overflow_eject_force(self, goal):
        if self._goal_is_long(goal):
            return LONG_GOAL_OVERFLOW_EJECT_FORCE_N, LONG_GOAL_OVERFLOW_EJECT_FORCE_MAX_N
        return MIDDLE_GOAL_OVERFLOW_EJECT_FORCE_N, MIDDLE_GOAL_OVERFLOW_EJECT_FORCE_MAX_N

    def _create_goal_mouth_gate(self, space, goal, inward: Vec2) -> GoalMouthGate:
        inward = inward.normalized()
        mouth_center = goal.center - (inward * goal.half_len)
        gate_thickness_px = GOAL_MOUTH_GATE_THICKNESS_PX
        gate_center = mouth_center - (
            inward * (GOAL_MOUTH_GATE_OUTSIDE_OFFSET_PX + (gate_thickness_px * 0.5))
        )
        gate_shape = space.create_static_box(
            (gate_center.x, gate_center.y),
            (goal.inner_width * GOAL_MOUTH_GATE_WIDTH_SCALE, gate_thickness_px),
            angle=self._tube_angle(goal.axis),
            friction=GOAL_MOUTH_GATE_FRICTION,
            restitution=GOAL_MOUTH_GATE_RESTITUTION,
            height_m=GOAL_MOUTH_GATE_HEIGHT_M,
            z_center_m=goal.floor_top_m + (GOAL_MOUTH_GATE_HEIGHT_M * 0.5),
            visual_rgba=None,
        )
        gate_shape.is_goal_surface = False
        gate_shape.is_goal_mouth_gate = True
        gate = GoalMouthGate(shape=gate_shape)
        self._set_mouth_gate_active(space, gate, False)
        return gate

    def _set_mouth_gate_active(self, space, gate: GoalMouthGate, active: bool):
        if bullet is None:
            return
        bullet.setCollisionFilterGroupMask(
            gate.shape.body.body_id,
            -1,
            collisionFilterGroup=GOAL_MOUTH_GATE_COLLISION_GROUP if active else 0,
            collisionFilterMask=BLOCK_COLLISION_GROUP if active else 0,
            physicsClientId=space.client_id,
        )

    def _activate_mouth_gate(self, space, zone: DumpZone):
        gate = self._mouth_gates.get((zone.goal_kind, zone.goal_index, zone.end))
        if gate is None:
            return
        gate.time_left = GOAL_MOUTH_GATE_ACTIVE_SEC
        self._set_mouth_gate_active(space, gate, True)

    def _register_goal_contact_bodies(self, goal):
        for body_id in goal.contact_body_ids:
            self._goal_by_contact_body_id[body_id] = goal

    def _goal_pack_origin_px(self, goal) -> float:
        pack_origin_px = MIDDLE_GOAL_PACKING_ORIGIN_PX
        if self._goal_is_long(goal):
            pack_origin_px = LONG_GOAL_PACKING_ORIGIN_PX
            pack_origin_px += LONG_GOAL_EMPTY_PACK_ORIGIN_BONUS_PX * ((1.0 - self._goal_fill_fraction(goal)) ** 1.6)
        else:
            pack_origin_px += MIDDLE_GOAL_EMPTY_PACK_ORIGIN_BONUS_PX * ((1.0 - self._goal_fill_fraction(goal)) ** 1.6)
        return max(GOAL_ENTRY_DEPTH_PX, pack_origin_px)

    def _goal_target_pitch_px(self, goal) -> float:
        if goal.capacity <= 1:
            return max(BLOCK_RADIUS_PX * 2.0, goal.half_len * 2.0)

        pack_origin_px = self._goal_pack_origin_px(goal)
        usable_length_px = max(
            BLOCK_RADIUS_PX * 2.0,
            (goal.half_len * 2.0) - pack_origin_px - GOAL_PACKING_TAIL_MARGIN_PX,
        )
        fill_ratio = LONG_GOAL_PACKING_FILL_RATIO
        if goal.capacity <= MIDDLE_GOAL_CAPACITY:
            fill_ratio = MIDDLE_GOAL_PACKING_FILL_RATIO
        return (usable_length_px / (goal.capacity - 1)) * fill_ratio

    def _register_goal_push(self, goal, mouth_center: Vec2, inward: Vec2, *, force_n: float):
        fill_fraction = self._goal_fill_fraction(goal)
        push_time_sec = GOAL_ENTRY_PUSH_TIME_SEC * (1.0 + (0.70 * fill_fraction))
        push_force_n = force_n * (1.0 + (0.25 * fill_fraction))
        goal.pack_mouth_center = mouth_center
        goal.pack_inward = inward.normalized()
        goal.pack_time_left = GOAL_PACKING_HOLD_SEC
        goal.active_pushes.append(
            GoalPush(
                mouth_center=mouth_center,
                inward=inward.normalized(),
                time_left=push_time_sec,
                force_n=push_force_n,
            )
        )
        if len(goal.active_pushes) > 4:
            goal.active_pushes = goal.active_pushes[-4:]

    def _spawn_into_goal(self, space, color: str, goal, inward: Vec2, *, entry_speed_pxps: float | None = None):
        inward = inward.normalized()
        mouth_center = goal.center - (inward * goal.half_len)
        side_jitter = goal.normal * random.uniform(-GOAL_ENTRY_LATERAL_JITTER_PX, GOAL_ENTRY_LATERAL_JITTER_PX)
        spawn_pos = mouth_center + (inward * GOAL_ENTRY_DEPTH_PX) + side_jitter

        shape = spawn_block(
            space,
            (spawn_pos.x, spawn_pos.y),
            color=color,
            air=False,
            elasticity=GOAL_SURFACE_RESTITUTION,
            z_center_m=goal.block_center_z_m,
        )

        main_speed = self._entry_speed_pxps(goal, entry_speed_pxps) * random.uniform(1.0 - GOAL_ENTRY_SPEED_JITTER, 1.0 + GOAL_ENTRY_SPEED_JITTER)
        side_speed = main_speed * GOAL_ENTRY_SIDE_SPEED_GAIN * random.uniform(-1.0, 1.0)
        shape.body.velocity = (inward * main_speed) + (goal.normal * side_speed)
        shape.body.angular_velocity = random.uniform(-GOAL_ENTRY_SPIN_RAD_S, GOAL_ENTRY_SPIN_RAD_S)
        shape.air_timer = 0.0
        self._register_goal_push(goal, mouth_center, inward, force_n=self._entry_push_force_n(goal))
        return shape

    def dump_into_goal(self, space, color: str, zone: DumpZone, *, entry_speed_pxps: float | None = None):
        self._activate_mouth_gate(space, zone)
        if zone.goal_kind == "long":
            goal = self.long_goals[zone.goal_index]
            inward = goal.axis if zone.end == "top" else (-goal.axis)
            self._spawn_into_goal(space, color, goal, inward, entry_speed_pxps=entry_speed_pxps)
        else:
            goal = self.middle_goals[zone.goal_index]
            inward = (-goal.axis) if zone.end == "A" else goal.axis
            self._spawn_into_goal(space, color, goal, inward, entry_speed_pxps=entry_speed_pxps)

    def _best_goal_match(self, shape, contact_ids, *, include_guides: bool):
        best_goal = None
        best_s = 0.0
        best_metric = None

        all_goals = [*self.long_goals, *self.middle_goals]
        for goal in all_goals:
            local_coords = self._shape_inside_goal_volume(shape, goal, include_guides=include_guides)
            if local_coords is None:
                continue

            axial, lateral = local_coords
            s = 0.5 + (axial / max(1e-6, goal.half_len * 2.0))
            has_shell_contact = bool(contact_ids) and bool(contact_ids & goal.contact_body_ids)
            metric = abs(lateral) + (0.15 * abs(axial)) + (0.0 if has_shell_contact else 5.0)
            if best_metric is None or metric < best_metric:
                best_metric = metric
                best_goal = goal
                best_s = max(0.0, min(1.0, s))

        return best_goal, best_s

    def _tracked_goal_for_shape(self, shape, contact_ids):
        return self._best_goal_match(shape, contact_ids, include_guides=True)

    def _goal_for_shape(self, shape, contact_ids):
        return self._best_goal_match(shape, contact_ids, include_guides=False)

    def _apply_goal_centering(self, goal):
        force_scale = 1.0 if self._goal_is_long(goal) else MIDDLE_GOAL_CENTERING_FORCE_SCALE
        if self._goal_impact_relax_timers.get(id(goal), 0.0) > 0.0:
            force_scale *= 0.15
        for shape in goal.tracked_shapes:
            body = shape.body
            axial, lateral = self._goal_local_coords(goal, body.position)
            if abs(axial) > (goal.half_len + goal.entry_guide_length_px + BLOCK_RADIUS_PX):
                continue

            lateral_speed_mps = pxps_to_mps(body.velocity.dot(goal.normal))
            lateral_m = px_to_meters(lateral)
            centering_force_n = -(
                (GOAL_CENTERING_STIFFNESS_N_PER_M * lateral_m)
                + (GOAL_CENTERING_DAMP_N_PER_MPS * lateral_speed_mps)
            )
            centering_force_n = clamp(
                centering_force_n,
                -GOAL_CENTERING_FORCE_MAX_N,
                GOAL_CENTERING_FORCE_MAX_N,
            ) * force_scale
            if abs(centering_force_n) < 1e-4:
                continue

            body.apply_force_at_world_point(
                goal.normal * newton_to_sim_force(centering_force_n),
                body.position,
            )

    def _apply_goal_axial_damping(self, goal):
        damp_scale = 1.0 if self._goal_is_long(goal) else MIDDLE_GOAL_AXIAL_DAMP_SCALE
        if self._goal_impact_relax_timers.get(id(goal), 0.0) > 0.0:
            damp_scale *= 0.10
        for shape in goal.tracked_shapes:
            body = shape.body
            axial_speed_mps = pxps_to_mps(body.velocity.dot(goal.axis))
            damp_force_n = clamp(
                -(GOAL_AXIAL_DAMP_N_PER_MPS * axial_speed_mps),
                -GOAL_AXIAL_DAMP_FORCE_MAX_N,
                GOAL_AXIAL_DAMP_FORCE_MAX_N,
            ) * damp_scale
            if abs(damp_force_n) < 1e-4:
                continue

            body.apply_force_at_world_point(
                goal.axis * newton_to_sim_force(damp_force_n),
                body.position,
            )

    def _set_shape_goal_contact_mode(self, space, shape, enabled: bool):
        if bullet is None:
            return

        current = getattr(shape, "_goal_contact_mode", None)
        if current is enabled:
            return

        if enabled:
            bullet.changeDynamics(
                shape.body.body_id,
                -1,
                lateralFriction=GOAL_BLOCK_CONTACT_FRICTION,
                rollingFriction=GOAL_BLOCK_CONTACT_ROLLING_FRICTION,
                spinningFriction=GOAL_BLOCK_CONTACT_SPINNING_FRICTION,
                restitution=GOAL_BLOCK_CONTACT_RESTITUTION,
                linearDamping=GOAL_BLOCK_LINEAR_DAMPING,
                angularDamping=GOAL_BLOCK_ANGULAR_DAMPING,
                physicsClientId=space.client_id,
            )
        else:
            bullet.changeDynamics(
                shape.body.body_id,
                -1,
                lateralFriction=float(getattr(shape, "friction", BLOCK_CONTACT_FRICTION)),
                rollingFriction=BLOCK_CONTACT_ROLLING_FRICTION,
                spinningFriction=BLOCK_CONTACT_SPINNING_FRICTION,
                restitution=float(getattr(shape, "elasticity", BLOCK_CONTACT_RESTITUTION)),
                linearDamping=0.0,
                angularDamping=0.0,
                physicsClientId=space.client_id,
            )

        shape._goal_contact_mode = enabled

    def _apply_goal_packing_assist(self, dt: float, goal):
        if self._goal_impact_relax_timers.get(id(goal), 0.0) > 0.0:
            goal.pack_time_left = max(0.0, goal.pack_time_left - dt)
            return

        if goal.pack_time_left <= 0.0 or goal.pack_inward is None or goal.pack_mouth_center is None:
            goal.pack_time_left = max(0.0, goal.pack_time_left - dt)
            return

        pack_origin_px = self._goal_pack_origin_px(goal)
        pitch_px = self._goal_target_pitch_px(goal)
        force_max_n = GOAL_PACKING_FORCE_MAX_N
        if not self._goal_is_long(goal):
            force_max_n *= MIDDLE_GOAL_PACKING_FORCE_SCALE

        ordered = []
        max_axial = (goal.half_len * 2.0) + goal.entry_guide_length_px + BLOCK_RADIUS_PX
        for shape in goal.tracked_shapes:
            rel = shape.body.position - goal.pack_mouth_center
            axial = rel.dot(goal.pack_inward)
            lateral = rel.dot(goal.normal)
            if axial < (-GOAL_ENTRY_PUSH_OUTSIDE_PX) or axial > max_axial:
                continue
            if abs(lateral) > (goal.inner_width * 0.56):
                continue
            ordered.append((axial, shape))

        ordered.sort(key=lambda item: item[0])

        packed_ordered = ordered[:goal.capacity]
        desired_last_axial = pack_origin_px + ((goal.capacity - 1) * pitch_px)
        base_eject_force_n, max_eject_force_n = self._goal_overflow_eject_force(goal)
        overflow_start_axial = desired_last_axial + (BLOCK_RADIUS_PX * 0.45)
        overflow_entries = [
            (axial, shape)
            for axial, shape in ordered[goal.capacity:]
            if axial > overflow_start_axial
        ]
        overflow_count = len(overflow_entries)
        if overflow_count > 0 and self._goal_is_long(goal):
            far_limit_px = (goal.half_len * 2.0) - LONG_GOAL_TAIL_BACKSTOP_REACH_PX
            for axial, shape in packed_ordered:
                if axial <= far_limit_px:
                    continue

                inward_speed_pxps = shape.body.velocity.dot(goal.pack_inward)
                if inward_speed_pxps <= 0.0:
                    continue
                overshoot_px = max(0.0, axial - far_limit_px)
                reverse_force_n = clamp(
                    LONG_GOAL_TAIL_BACKSTOP_FORCE_N
                    + (px_to_meters(overshoot_px) * 7.0)
                    + (pxps_to_mps(max(0.0, inward_speed_pxps)) * 0.55),
                    0.0,
                    LONG_GOAL_TAIL_BACKSTOP_FORCE_MAX_N,
                )
                shape.body.apply_force_at_world_point(
                    (-goal.pack_inward) * newton_to_sim_force(reverse_force_n),
                    shape.body.position,
                )
        elif overflow_count > 0:
            far_limit_px = (goal.half_len * 2.0) - MIDDLE_GOAL_TAIL_BACKSTOP_REACH_PX
            for axial, shape in packed_ordered:
                if axial <= far_limit_px:
                    continue

                inward_speed_pxps = shape.body.velocity.dot(goal.pack_inward)
                if inward_speed_pxps <= 0.0:
                    continue
                overshoot_px = max(0.0, axial - far_limit_px)
                reverse_force_n = clamp(
                    MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_N
                    + (px_to_meters(overshoot_px) * 28.0)
                    + (pxps_to_mps(max(0.0, inward_speed_pxps)) * 1.9),
                    0.0,
                    MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_MAX_N * 1.25,
                )
                shape.body.apply_force_at_world_point(
                    (-goal.pack_inward) * newton_to_sim_force(reverse_force_n),
                    shape.body.position,
                )

        if overflow_count <= 0:
            goal.pack_time_left = max(0.0, goal.pack_time_left - dt)
            return

        for axial, shape in overflow_entries:
            overflow_depth_px = max(0.0, axial - overflow_start_axial)
            eject_gain = 0.75 + (
                0.25
                * clamp(
                    overflow_depth_px / max(1e-6, BLOCK_RADIUS_PX),
                    0.0,
                    1.0,
                )
            )
            eject_force_n = clamp(
                base_eject_force_n * eject_gain,
                0.0,
                max_eject_force_n,
            )
            shape.body.apply_force_at_world_point(
                goal.pack_inward * newton_to_sim_force(eject_force_n),
                shape.body.position,
            )
        if self._goal_is_long(goal):
            goal.pack_time_left = max(0.0, goal.pack_time_left - dt)
            return

        for slot_idx, (axial, shape) in enumerate(packed_ordered[:goal.capacity]):
            desired_axial = pack_origin_px + (slot_idx * pitch_px)
            error_px = desired_axial - axial
            if error_px < 0.0 and overflow_count <= 0:
                continue
            if abs(error_px) < 1e-4:
                continue

            speed_mps = pxps_to_mps(shape.body.velocity.dot(goal.pack_inward))
            assist_force_n = (
                (GOAL_PACKING_STIFFNESS_N_PER_M * px_to_meters(error_px))
                - (GOAL_PACKING_DAMP_N_PER_MPS * speed_mps)
            )
            assist_force_n = clamp(
                assist_force_n,
                0.0,
                force_max_n,
            )
            if abs(assist_force_n) < 1e-4:
                continue

            shape.body.apply_force_at_world_point(
                goal.pack_inward * newton_to_sim_force(assist_force_n),
                shape.body.position,
            )

        goal.pack_time_left = max(0.0, goal.pack_time_left - dt)

    def _apply_goal_pushes(self, dt: float, space, goal):
        if not goal.active_pushes:
            return

        survivors = []
        for push in goal.active_pushes:
            fade = clamp(push.time_left / GOAL_ENTRY_PUSH_TIME_SEC, 0.0, 1.0)
            overflow_count = max(0, len(goal.tracked_shapes) - goal.capacity)
            mouth_push_scale = self._goal_overflow_mouth_push_boost(goal) if overflow_count > 0 else 1.0
            candidates = []
            for shape in goal.tracked_shapes:
                rel = shape.body.position - push.mouth_center
                axial = rel.dot(push.inward)
                lateral = rel.dot(goal.normal)
                if axial < (-GOAL_ENTRY_PUSH_OUTSIDE_PX) or axial > GOAL_ENTRY_PUSH_REACH_PX:
                    continue
                if abs(lateral) > (goal.inner_width * 0.55):
                    continue
                candidates.append((axial, shape))

            candidates.sort(key=lambda item: item[0])

            for idx, (axial, shape) in enumerate(candidates[:4]):
                axial_gain = 1.0 - clamp(axial / max(1e-6, GOAL_ENTRY_PUSH_REACH_PX), 0.0, 1.0)
                chain_decay = 0.62 ** idx
                force_n = (
                    push.force_n
                    * (0.35 + (0.65 * fade))
                    * (0.45 + (0.55 * axial_gain))
                    * mouth_push_scale
                    * chain_decay
                )
                if force_n < 1e-4:
                    continue
                shape.body.apply_force_at_world_point(
                    push.inward * newton_to_sim_force(force_n),
                    shape.body.position,
                )

                inward_speed_pxps = shape.body.velocity.dot(push.inward)
                if axial <= GOAL_MOUTH_BACKSTOP_REACH_PX and inward_speed_pxps < 0.0:
                    reverse_speed_mps = pxps_to_mps(-inward_speed_pxps)
                    backstop_force_n = clamp(
                        GOAL_MOUTH_BACKSTOP_FORCE_N + (reverse_speed_mps * 1.6),
                        0.0,
                        GOAL_MOUTH_BACKSTOP_FORCE_MAX_N,
                    ) * (0.35 + (0.65 * fade))
                    shape.body.apply_force_at_world_point(
                        push.inward * newton_to_sim_force(backstop_force_n),
                        shape.body.position,
                    )

                if (
                    overflow_count > 0
                    and (not self._goal_is_long(goal))
                    and axial >= ((goal.half_len * 2.0) - MIDDLE_GOAL_TAIL_BACKSTOP_REACH_PX)
                    and inward_speed_pxps > 0.0
                ):
                    penetration_ratio = clamp(
                        (axial - ((goal.half_len * 2.0) - MIDDLE_GOAL_TAIL_BACKSTOP_REACH_PX))
                        / max(1e-6, MIDDLE_GOAL_TAIL_BACKSTOP_REACH_PX),
                        0.0,
                        1.0,
                    )
                    tail_force_n = clamp(
                        MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_N
                        + (pxps_to_mps(inward_speed_pxps) * (1.1 + (0.9 * penetration_ratio))),
                        0.0,
                        MIDDLE_GOAL_TAIL_BACKSTOP_FORCE_MAX_N,
                    )
                    shape.body.apply_force_at_world_point(
                        (-push.inward) * newton_to_sim_force(tail_force_n),
                        shape.body.position,
                    )

            push.time_left = max(0.0, push.time_left - dt)
            if push.time_left > 0.0:
                survivors.append(push)

        goal.active_pushes = survivors

    def _apply_robot_goal_impacts(self, space, robot):
        if bullet is None or robot is None or not self._goal_by_contact_body_id:
            return

        robot_body_id = robot.body.body_id
        robot_speed_pxps = robot.body.velocity.length
        if robot_speed_pxps < 1e-4:
            return

        contacts = bullet.getContactPoints(bodyA=robot_body_id, physicsClientId=space.client_id)
        impacts: dict[int, dict[str, object]] = {}
        for cp in contacts:
            other_id = cp[2] if cp[1] == robot_body_id else cp[1]
            goal = self._goal_by_contact_body_id.get(other_id)
            if goal is None:
                continue

            normal_force_n = float(cp[9])
            if normal_force_n < GOAL_IMPACT_MIN_NORMAL_FORCE_N:
                continue

            impact_normal = Vec2(cp[7][0], cp[7][1])
            closing_speed_mps = pxps_to_mps(abs(robot.body.velocity.dot(impact_normal)))
            if closing_speed_mps < GOAL_IMPACT_MIN_CLOSING_SPEED_MPS:
                continue

            info = impacts.setdefault(
                id(goal),
                {
                    "goal": goal,
                    "normal_force_n": 0.0,
                    "closing_speed_mps": 0.0,
                    "impact_dir": Vec2(0.0, 0.0),
                },
            )
            info["normal_force_n"] = max(float(info["normal_force_n"]), normal_force_n)
            info["closing_speed_mps"] = max(float(info["closing_speed_mps"]), closing_speed_mps)
            info["impact_dir"] = info["impact_dir"] + (-impact_normal)

        for info in impacts.values():
            goal = info["goal"]
            if not goal.tracked_shapes:
                continue

            goal_key = id(goal)
            if self._goal_impact_cooldowns.get(goal_key, 0.0) > 0.0:
                continue

            impact_dir = info["impact_dir"]
            if impact_dir.length < 1e-6:
                impact_dir = robot.body.velocity.normalized()

            axis_component = goal.axis * impact_dir.dot(goal.axis)
            lateral_component = goal.normal * impact_dir.dot(goal.normal)
            shake_dir = (
                (axis_component * GOAL_IMPACT_AXIS_SHAKE_BLEND)
                + (lateral_component * GOAL_IMPACT_LATERAL_SHAKE_BLEND)
            )
            if shake_dir.length < 1e-6:
                shake_dir = goal.axis
            shake_dir = shake_dir.normalized()

            impact_force_n = clamp(
                (float(info["normal_force_n"]) * GOAL_IMPACT_FORCE_SCALE)
                + (float(info["closing_speed_mps"]) * GOAL_IMPACT_SPEED_FORCE_SCALE),
                0.0,
                GOAL_IMPACT_FORCE_MAX_N,
            ) * GOAL_IMPACT_SHAKE_SCALE
            if impact_force_n < 1e-4:
                continue

            ordered_shapes = []
            for shape in goal.tracked_shapes:
                axial, _ = self._goal_local_coords(goal, shape.body.position)
                ordered_shapes.append((abs(axial), axial, shape))
            ordered_shapes.sort(key=lambda item: item[0], reverse=True)

            for idx, (_, axial, shape) in enumerate(ordered_shapes):
                depth_scale = GOAL_IMPACT_DEPTH_DECAY ** idx
                shape_force_n = impact_force_n * depth_scale
                lateral_sign = -1.0 if (shape.body.body_id % 2 == 0) else 1.0
                total_force = (
                    shake_dir * newton_to_sim_force(shape_force_n)
                    + goal.normal * newton_to_sim_force(shape_force_n * GOAL_IMPACT_LATERAL_JITTER_RATIO * lateral_sign)
                )
                shape.body.apply_force_at_world_point(total_force, shape.body.position)

                if abs(axial) > (goal.half_len * 0.72):
                    outward_dir = goal.axis if axial > 0.0 else (-goal.axis)
                    shape.body.apply_force_at_world_point(
                        outward_dir * newton_to_sim_force(shape_force_n * GOAL_IMPACT_END_EJECT_RATIO),
                        shape.body.position,
                    )

                bullet.applyExternalForce(
                    shape.body.body_id,
                    -1,
                    [0.0, 0.0, min(GOAL_IMPACT_Z_KICK_MAX_N, shape_force_n * GOAL_IMPACT_Z_KICK_RATIO)],
                    [
                        px_to_meters(shape.body.position.x),
                        px_to_meters(shape.body.position.y),
                        shape.body.z,
                    ],
                    bullet.WORLD_FRAME,
                    physicsClientId=space.client_id,
                )

            self._goal_impact_cooldowns[goal_key] = GOAL_IMPACT_COOLDOWN_SEC
            self._goal_impact_relax_timers[goal_key] = GOAL_IMPACT_RELAX_SEC

    def update(self, dt: float, space, robot=None):
        if self._mouth_gates:
            for gate in self._mouth_gates.values():
                if gate.time_left <= 0.0:
                    continue
                gate.time_left = max(0.0, gate.time_left - dt)
                if gate.time_left <= 0.0:
                    self._set_mouth_gate_active(space, gate, False)

        if self._goal_impact_cooldowns:
            expired_keys = []
            for goal_key, time_left in self._goal_impact_cooldowns.items():
                new_time_left = time_left - dt
                if new_time_left <= 0.0:
                    expired_keys.append(goal_key)
                else:
                    self._goal_impact_cooldowns[goal_key] = new_time_left
            for goal_key in expired_keys:
                del self._goal_impact_cooldowns[goal_key]

        if self._goal_impact_relax_timers:
            expired_keys = []
            for goal_key, time_left in self._goal_impact_relax_timers.items():
                new_time_left = time_left - dt
                if new_time_left <= 0.0:
                    expired_keys.append(goal_key)
                else:
                    self._goal_impact_relax_timers[goal_key] = new_time_left
            for goal_key in expired_keys:
                del self._goal_impact_relax_timers[goal_key]

        empty_contact_ids = frozenset()
        for shape in getattr(space, "block_shapes", []):
            shape.goal_scored = False
            shape.goal_protected = False

        for goal in self.long_goals:
            goal.queue.clear()
            goal.tracked_shapes.clear()
        for goal in self.middle_goals:
            goal.queue.clear()
            goal.tracked_shapes.clear()

        for shape in getattr(space, "block_shapes", []):
            tracked_goal, _ = self._tracked_goal_for_shape(shape, empty_contact_ids)
            if tracked_goal is not None:
                tracked_goal.tracked_shapes.append(shape)
                shape.goal_protected = True

            goal, s = self._goal_for_shape(shape, empty_contact_ids)
            if goal is None:
                continue

            goal.queue.append(
                GoalBall(
                    shape=shape,
                    color=getattr(shape, "block_color", "red"),
                    s=s,
                )
            )
            shape.goal_scored = True

        for shape in getattr(space, "block_shapes", []):
            self._set_shape_goal_contact_mode(space, shape, getattr(shape, "goal_protected", False))

        for goal in self.long_goals:
            goal.queue.sort(key=lambda ball: ball.s)
            self._apply_goal_centering(goal)
            self._apply_goal_axial_damping(goal)
            self._apply_goal_packing_assist(dt, goal)
            self._apply_goal_pushes(dt, space, goal)
        for goal in self.middle_goals:
            goal.queue.sort(key=lambda ball: ball.s)
            self._apply_goal_centering(goal)
            self._apply_goal_axial_damping(goal)
            self._apply_goal_packing_assist(dt, goal)
            self._apply_goal_pushes(dt, space, goal)

        self._apply_robot_goal_impacts(space, robot)
