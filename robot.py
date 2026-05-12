import math
import random
from dataclasses import dataclass, replace

from vec2 import Vec2

from config import (
    FIELD_SIZE_PX,
    ROBOT_SIZE_IN,
    ROBOT_DEFAULT_MASS_KG,
    DRIVETRAIN_DEFAULT_RPM,
    DRIVETRAIN_MOTOR_COUNT,
    DRIVETRAIN_MOTOR_POWER_W,
    DRIVE_WHEEL_DEFAULT_DIAMETER_IN,
    DRIVE_TRACK_WIDTH_IN,
    DRIVE_MOTOR_EFFICIENCY,
    DRIVE_INPUT_TURN_GAIN,
    DRIVE_SPEED_CONTROL_TIME_CONSTANT,
    DRIVE_ACTIVE_BRAKE_FACTOR,
    DRIVE_LINEAR_DRAG_COEFF,
    DRIVE_ANGULAR_DRAG_COEFF,
    DRIVE_LONGITUDINAL_MU,
    DRIVE_LATERAL_MU,
    DRIVE_LATERAL_TIME_CONSTANT_SEC,
    DRIVE_ROLLING_RESISTANCE_COEFF,
    DRIVE_YAW_SCRUB_COEFF,
    DRIVE_YAW_VISCOUS_DAMPING,
    DRIVE_MOVING_TURN_REDUCTION,
    DRIVE_MOVING_LATERAL_HOLD_REDUCTION,
    STD_GRAVITY,
    INTAKE_CAPACITY,
    INTAKE_RPM,
    BLOCK_RADIUS_PX,
    LOADER_PADDLE_LEN_PX,
    LOADER_PADDLE_W_PX,
    LOADER_PADDLE_T_PX,
    clamp,
    lerp,
    rpm01,
    inches_to_px,
    inches_to_m,
    mps_to_pxps,
    px_to_meters,
    pxps_to_mps,
    drivetrain_free_speed_mps,
)
from physics import ROBOT_CHASSIS_COLLISION_GROUP, bullet, spawn_block


ROBOT_REBOUND_FORCE_THRESHOLD_N = 42.0
ROBOT_REBOUND_SPEED_THRESHOLD_MPS = 0.18
ROBOT_REBOUND_TIME_SEC = 0.14
ROBOT_REBOUND_IMPULSE_FORCE_N = 78.0
ROBOT_REBOUND_IMPULSE_FORCE_MAX_N = 190.0
ROBOT_REBOUND_DRAG_SCALE = 0.08
ROBOT_REBOUND_LATERAL_HOLD_SCALE = 0.05
ROBOT_REBOUND_YAW_DAMP_SCALE = 0.18
ROBOT_BLOCK_CONTACT_DOWNFORCE_N = 80.0
ROBOT_BLOCK_CONTACT_DOWNFORCE_MAX_N = 180.0

PARK_ZONE_STEP_SPEED_THRESHOLD_MPS = 0.0
PARK_ZONE_STEP_FORCE_THRESHOLD_N = 0.05
PARK_ZONE_STEP_ASSIST_FORCE_N = 52.0
PARK_ZONE_STEP_ASSIST_FORCE_MAX_N = 110.0
PARK_ZONE_ENTRY_INWARD_PUSH_N = 28.0
PARK_ZONE_ENTRY_INWARD_PUSH_MAX_N = 58.0
PARK_ZONE_STEP_COOLDOWN_SEC = 0.02
PARK_ZONE_MAX_TILT_RAD = math.radians(32.0)
PARK_ZONE_EXIT_ASSIST_FORCE_N = 64.0
PARK_ZONE_EXIT_ASSIST_FORCE_MAX_N = 128.0
PARK_ZONE_EXIT_OUTWARD_PUSH_N = 42.0
PARK_ZONE_CONTACT_ASSIST_ENABLED = False
PARK_ZONE_CONTACT_STABILITY_HOLD_SCALE = 1.45
PARK_ZONE_CONTACT_STABILITY_YAW_DAMP_SCALE = 2.10
PARK_ZONE_CONTACT_STABILITY_TURN_SCALE = 0.82
PARK_ZONE_CONTACT_STABILITY_TIME_SEC = 0.12
PARK_ZONE_CONTACT_SPEED_LIMIT_SCALE = 1.18
PARK_ZONE_CONTACT_YAW_RATE_LIMIT_RAD_S = math.radians(90.0)
PARK_ZONE_ENTER_TANGENT_SPEED_MAX_MPS = 0.16
PARK_ZONE_ENTER_YAW_DAMP_SCALE = 0.32
PARK_ZONE_ENTER_LIFT_FORCE_N = 66.0
PARK_ZONE_ENTER_LIFT_FORCE_MAX_N = 140.0
PARK_ZONE_ENTER_INWARD_PUSH_N = 100.0
PARK_ZONE_ENTER_INWARD_PUSH_MAX_N = 210.0
PARK_ZONE_ENTER_MIN_INWARD_SPEED_MPS = 0.55
PARK_ZONE_ENTER_MAX_INWARD_SPEED_MPS = 1.05
PARK_ZONE_ENTER_ASSIST_TURN_LIMIT = 0.70

ROBOT_TILT_STIFFNESS_NM = 22.0
ROBOT_TILT_DAMPING_NM = 5.4
ROBOT_TILT_TORQUE_MAX_NM = 18.0
ROBOT_WHEELBASE_SCALE = 0.68
ROBOT_WHEEL_CONTACT_Z_M = 0.018
ROBOT_WHEEL_AIRBORNE_Z_M = 0.34
ROBOT_WHEEL_DRIVE_DISABLE_TILT_RAD = math.radians(66.0)
ROBOT_WHEEL_DRIVE_FADE_TILT_RAD = math.radians(30.0)
ROBOT_PADDLE_DEPLOY_RATE = 7.5
ROBOT_PADDLE_RETRACT_RATE = 10.0
ROBOT_PADDLE_LOAD_FRACTION = 0.86
ROBOT_PADDLE_COLLISION_FRACTION = 0.92
ROBOT_PADDLE_BLOCKED_FRACTION = 0.34
ROBOT_PADDLE_CLEARANCE_MARGIN_PX = 2.0
ROBOT_PADDLE_LOADER_LOAD_FRACTION = 0.24


def _sign(x):
    return -1.0 if x < 0.0 else 1.0


def _polygon_axes(points):
    axes = []
    count = len(points)
    for idx, point in enumerate(points):
        edge = points[(idx + 1) % count] - point
        normal = edge.perpendicular().normalized()
        if normal.length > 1e-6:
            axes.append(normal)
    return axes


def _project_points(points, axis):
    dots = [point.dot(axis) for point in points]
    return min(dots), max(dots)


def _polygons_overlap(poly_a, poly_b, axes_a=None):
    axes = list(axes_a or _polygon_axes(poly_a))
    axes.extend(_polygon_axes(poly_b))
    for axis in axes:
        min_a, max_a = _project_points(poly_a, axis)
        min_b, max_b = _project_points(poly_b, axis)
        if max_a < min_b or max_b < min_a:
            return False
    return True


def _circle_overlaps_poly(center, radius, poly, axes=None):
    axes_to_test = list(axes or _polygon_axes(poly))
    closest = min(poly, key=lambda point: (point - center).length_sq)
    radial = (closest - center).normalized()
    if radial.length > 1e-6:
        axes_to_test.append(radial)
    for axis in axes_to_test:
        min_p, max_p = _project_points(poly, axis)
        c = center.dot(axis)
        if c + radius < min_p or c - radius > max_p:
            return False
    return True


@dataclass(frozen=True)
class RobotDriveConfig:
    width_in: float = ROBOT_SIZE_IN
    length_in: float = ROBOT_SIZE_IN
    mass_kg: float = ROBOT_DEFAULT_MASS_KG
    drivetrain_rpm: float = DRIVETRAIN_DEFAULT_RPM
    intake_rpm: float = float(INTAKE_RPM)
    wheel_diameter_in: float = DRIVE_WHEEL_DEFAULT_DIAMETER_IN
    outtake_from_back: bool = False

    motor_count: int = DRIVETRAIN_MOTOR_COUNT
    motor_power_w: float = DRIVETRAIN_MOTOR_POWER_W
    motor_efficiency: float = DRIVE_MOTOR_EFFICIENCY

    track_width_in: float = DRIVE_TRACK_WIDTH_IN
    turn_input_gain: float = DRIVE_INPUT_TURN_GAIN
    speed_controller_time_constant: float = DRIVE_SPEED_CONTROL_TIME_CONSTANT
    active_brake_factor: float = DRIVE_ACTIVE_BRAKE_FACTOR

    longitudinal_mu: float = DRIVE_LONGITUDINAL_MU
    lateral_mu: float = DRIVE_LATERAL_MU
    lateral_time_constant: float = DRIVE_LATERAL_TIME_CONSTANT_SEC
    rolling_resistance_coeff: float = DRIVE_ROLLING_RESISTANCE_COEFF

    linear_drag_coeff: float = DRIVE_LINEAR_DRAG_COEFF
    angular_drag_coeff: float = DRIVE_ANGULAR_DRAG_COEFF
    yaw_scrub_coeff: float = DRIVE_YAW_SCRUB_COEFF
    yaw_viscous_damping: float = DRIVE_YAW_VISCOUS_DAMPING
    moving_turn_reduction: float = DRIVE_MOVING_TURN_REDUCTION
    moving_lateral_hold_reduction: float = DRIVE_MOVING_LATERAL_HOLD_REDUCTION

    def __post_init__(self):
        if self.width_in <= 0.0:
            raise ValueError("width_in must be > 0")
        if self.length_in <= 0.0:
            raise ValueError("length_in must be > 0")
        if self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be > 0")
        if self.drivetrain_rpm <= 0.0:
            raise ValueError("drivetrain_rpm must be > 0")
        if self.intake_rpm <= 0.0:
            raise ValueError("intake_rpm must be > 0")
        if self.wheel_diameter_in <= 0.0:
            raise ValueError("wheel_diameter_in must be > 0")
        if self.motor_count <= 0:
            raise ValueError("motor_count must be > 0")
        if self.motor_power_w <= 0.0:
            raise ValueError("motor_power_w must be > 0")
        if self.track_width_in <= 0.0:
            raise ValueError("track_width_in must be > 0")
        if self.speed_controller_time_constant <= 0.0:
            raise ValueError("speed_controller_time_constant must be > 0")
        if self.lateral_time_constant <= 0.0:
            raise ValueError("lateral_time_constant must be > 0")
        if self.yaw_viscous_damping < 0.0:
            raise ValueError("yaw_viscous_damping must be >= 0")
        if not (0.0 <= self.moving_turn_reduction <= 0.95):
            raise ValueError("moving_turn_reduction must be between 0 and 0.95")
        if not (0.0 <= self.moving_lateral_hold_reduction <= 0.95):
            raise ValueError("moving_lateral_hold_reduction must be between 0 and 0.95")

    @property
    def width_px(self) -> float:
        return float(inches_to_px(self.width_in))

    @property
    def length_px(self) -> float:
        return float(inches_to_px(self.length_in))

    @property
    def wheel_radius_m(self) -> float:
        return inches_to_m(self.wheel_diameter_in) * 0.5

    @property
    def track_width_px(self) -> float:
        return float(inches_to_px(self.track_width_in))

    @property
    def track_width_m(self) -> float:
        return inches_to_m(self.track_width_in)

    @property
    def wheel_free_omega_rad_s(self) -> float:
        return self.drivetrain_rpm * math.tau / 60.0

    @property
    def wheel_free_speed_mps(self) -> float:
        return drivetrain_free_speed_mps(self.drivetrain_rpm, self.wheel_diameter_in)

    @property
    def motors_per_side(self) -> float:
        return max(0.5, self.motor_count / 2.0)

    @property
    def motor_stall_torque_nm(self) -> float:
        return 4.0 * (self.motor_power_w * self.motor_efficiency) / max(self.wheel_free_omega_rad_s, 1e-6)

    @property
    def side_stall_force_n(self) -> float:
        return (self.motor_stall_torque_nm * self.motors_per_side) / max(self.wheel_radius_m, 1e-6)

    @property
    def side_normal_force_n(self) -> float:
        return 0.5 * self.mass_kg * STD_GRAVITY

    @property
    def side_longitudinal_traction_n(self) -> float:
        return self.longitudinal_mu * self.side_normal_force_n

    @property
    def side_lateral_traction_n(self) -> float:
        return self.lateral_mu * self.side_normal_force_n

    @property
    def side_rolling_resistance_n(self) -> float:
        return self.rolling_resistance_coeff * self.side_normal_force_n

    @property
    def total_stall_force_n(self) -> float:
        return 2.0 * self.side_stall_force_n

    @property
    def total_longitudinal_traction_n(self) -> float:
        return self.longitudinal_mu * self.mass_kg * STD_GRAVITY

    @property
    def total_lateral_traction_n(self) -> float:
        return self.lateral_mu * self.mass_kg * STD_GRAVITY

    @property
    def total_rolling_resistance_n(self) -> float:
        return self.rolling_resistance_coeff * self.mass_kg * STD_GRAVITY

    @property
    def max_drive_accel_mps2(self) -> float:
        return min(self.total_stall_force_n, self.total_longitudinal_traction_n) / max(1e-6, self.mass_kg)

    @property
    def max_brake_accel_mps2(self) -> float:
        brake_force_n = min(
            self.total_stall_force_n * self.active_brake_factor,
            self.total_longitudinal_traction_n,
        )
        return brake_force_n / max(1e-6, self.mass_kg)

    @property
    def neutral_brake_force_n(self) -> float:
        return min(
            self.total_stall_force_n * 0.24,
            self.total_longitudinal_traction_n * 0.50,
        )

    @property
    def max_theoretical_yaw_rate_rad_s(self) -> float:
        return (2.0 * self.wheel_free_speed_mps) / max(self.track_width_m, 1e-6)

    @property
    def intake_ratio(self) -> float:
        return rpm01(self.intake_rpm, 300.0, 1500.0)

    @property
    def intake_time_sec(self) -> float:
        return 0.5 * lerp(0.85, 0.18, self.intake_ratio)

    @property
    def intake_jitter_sec(self) -> float:
        return 0.12 * self.intake_time_sec

    @property
    def dump_interval_sec(self) -> float:
        return lerp(0.34, 0.09, self.intake_ratio)

    @property
    def dump_floor_speed_pxps(self) -> float:
        return lerp(35.0, 120.0, self.intake_ratio)


class Robot:
    def __init__(
        self,
        space,
        pos_px,
        *,
        drive_config: RobotDriveConfig | None = None,
        mass_kg: float | None = None,
        drivetrain_rpm: float | None = None,
        wheel_diameter_in: float | None = None,
    ):
        if drive_config is None:
            drive_config = RobotDriveConfig()

        overrides = {}
        if mass_kg is not None:
            overrides["mass_kg"] = mass_kg
        if drivetrain_rpm is not None:
            overrides["drivetrain_rpm"] = drivetrain_rpm
        if wheel_diameter_in is not None:
            overrides["wheel_diameter_in"] = wheel_diameter_in
        if overrides:
            drive_config = replace(drive_config, **overrides)

        self.drive = drive_config

        w = self.drive.width_px
        h = self.drive.length_px
        self.width_px = w
        self.length_px = h

        self.shape = space.create_robot_box(
            pos_px,
            (w, h),
            mass_kg=self.drive.mass_kg,
            friction=0.26,
            restitution=0.32,
            wheel_radius_m=self.drive.wheel_radius_m,
            track_width_m=self.drive.track_width_m,
        )
        self.body = self.shape.body
        if bullet is not None:
            bullet.changeDynamics(
                self.body.body_id,
                -1,
                linearDamping=0.02,
                angularDamping=0.18,
                physicsClientId=space.client_id,
            )

        self.storage = []
        self.current_pickup = None

        self.cmd_forward = 0.0
        self.cmd_turn = 0.0
        self.intake_enabled = False
        self.loader_paddle_requested = False
        self.loader_paddle_active = False
        self.loader_paddle_fraction = 0.0
        self._set_loader_paddle_collision_active(False)

        self.dump_active = False
        self.dump_hold_active = False
        self.dump_timer = 0.0
        self.rebound_timer = 0.0
        self.parking_step_cooldown = 0.0
        self.parking_contact_timer = 0.0

        self.drive_debug = {
            "speed_mps": 0.0,
            "forward_speed_mps": 0.0,
            "lateral_speed_mps": 0.0,
            "yaw_rate_deg_s": 0.0,
            "left_cmd": 0.0,
            "right_cmd": 0.0,
        }

    # ---------------- basis ----------------

    def front_vec(self):
        return Vec2(0.0, -1.0).rotated(self.body.angle)

    def right_vec(self):
        return Vec2(1.0, 0.0).rotated(self.body.angle)

    def front_point(self, extra=0.0):
        return self.body.position + self.front_vec() * (self.length_px / 2 + extra)

    def dump_vec(self):
        return -self.front_vec() if self.drive.outtake_from_back else self.front_vec()

    def dump_point(self, extra=0.0):
        return self.body.position + self.dump_vec() * (self.length_px / 2 + extra)

    # ---------------- input ----------------

    def set_input(self, forward, turn, intake_enabled, loader_paddle_active):
        self.cmd_forward = clamp(forward, -1.0, 1.0)
        self.cmd_turn = -clamp(turn, -1.0, 1.0)
        self.intake_enabled = bool(intake_enabled)
        self.loader_paddle_requested = bool(loader_paddle_active)

    # ---------------- dump toggle ----------------

    def toggle_dump(self):
        self.dump_active = not self.dump_active
        if not (self.dump_active or self.dump_hold_active):
            self.dump_timer = 0.0

    def set_dump_hold(self, active: bool):
        self.dump_hold_active = bool(active)
        if not (self.dump_active or self.dump_hold_active):
            self.dump_timer = 0.0

    def _set_loader_paddle_collision_active(self, active: bool):
        if bullet is None:
            return

        link_indices = getattr(self.shape, "loader_paddle_link_indices", ())
        if not link_indices:
            return

        group = ROBOT_CHASSIS_COLLISION_GROUP if active else 0
        mask = 0xFFFF if active else 0
        for link_index in link_indices:
            bullet.setCollisionFilterGroupMask(
                self.body.body_id,
                link_index,
                collisionFilterGroup=group,
                collisionFilterMask=mask,
                physicsClientId=self.body.world.client_id,
            )
        self.shape.loader_paddle_collision_active = bool(active)

    # ---------------- pickup ----------------

    def can_accept_block(self):
        return len(self.storage) < INTAKE_CAPACITY and self.current_pickup is None

    def start_pickup_color(self, color: str):
        if not self.can_accept_block():
            return False
        t = self.drive.intake_time_sec + random.uniform(
            -self.drive.intake_jitter_sec,
            self.drive.intake_jitter_sec,
        )
        t = max(0.05, t)
        self.current_pickup = {"time": t, "color": color}
        return True

    def _update_pickup(self, dt: float):
        if self.current_pickup is None:
            return
        self.current_pickup["time"] -= dt
        if self.current_pickup["time"] <= 0.0:
            if len(self.storage) < INTAKE_CAPACITY:
                self.storage.append(self.current_pickup["color"])
            self.current_pickup = None

    # ---------------- intake from field ----------------

    def try_intake_from_field(self, space):
        if not self.intake_enabled or not self.can_accept_block():
            return

        fwd = self.front_vec()
        right = self.right_vec()
        best = None
        best_metric = 1e9

        for shape in space.block_shapes:
            if getattr(shape, "goal_scored", False) or getattr(shape, "goal_protected", False):
                continue
            pos = shape.body.position
            vec = pos - self.body.position
            dist = vec.length
            pickup_radius = float(getattr(shape, "block_pickup_radius_px", BLOCK_RADIUS_PX))
            if dist < 1e-3:
                continue
            front_offset = vec.dot(fwd)
            side_offset = abs(vec.dot(right))
            max_front = (self.length_px * 0.5) + pickup_radius + 12.0
            max_side = (self.width_px * 0.38) + pickup_radius
            if front_offset < (-pickup_radius * 0.35) or front_offset > max_front:
                continue
            if side_offset > max_side:
                continue
            metric = front_offset + (side_offset * 0.35)
            if metric < best_metric:
                best_metric = metric
                best = shape

        if best is None:
            return

        color = getattr(best, "block_color", "red")
        if best.body in space.bodies:
            space.remove(best, best.body)
        self.start_pickup_color(color)

    # ---------------- dump ----------------

    def _dump_to_floor(self, space, color: str):
        fwd = self.dump_vec()
        pos = self.dump_point(extra=BLOCK_RADIUS_PX + 4)
        shape = spawn_block(space, (pos.x, pos.y), color=color, air=True)
        shape.body.velocity = fwd.normalized() * float(self.drive.dump_floor_speed_pxps)

    def _update_dump(self, dt: float, space, goals):
        if not (self.dump_active or self.dump_hold_active) or len(self.storage) == 0:
            self.dump_timer = 0.0
            return

        if self.dump_timer > 0.0:
            self.dump_timer -= dt
            return

        zone_probe_points = [
            self.dump_point(extra=0),
            self.dump_point(extra=5),
            self.dump_point(extra=12),
        ]
        zone = goals.find_zone_for_points(zone_probe_points)

        color = self.storage.pop(0)

        if zone is None:
            self._dump_to_floor(space, color)
        else:
            goals.dump_into_goal(space, color, zone, entry_speed_pxps=self.drive.dump_floor_speed_pxps)

        self.dump_timer = self.drive.dump_interval_sec

    # ---------------- loader paddle geometry ----------------

    def paddle_geometry(self, *, require_loaded=True):
        if require_loaded and self.loader_paddle_fraction < ROBOT_PADDLE_LOAD_FRACTION:
            return None

        fwd = self.front_vec()
        right = self.right_vec()

        center = self.body.position + fwd * (self.length_px / 2 + LOADER_PADDLE_LEN_PX * 0.55)

        half_w = LOADER_PADDLE_W_PX * 0.5
        half_t = LOADER_PADDLE_T_PX * 0.5

        p1 = center + right * half_w + fwd * half_t
        p2 = center - right * half_w + fwd * half_t
        p3 = center - right * half_w - fwd * half_t
        p4 = center + right * half_w - fwd * half_t
        rect_pts = [p1, p2, p3, p4]

        robot_front_center = self.body.position + fwd * (self.length_px / 2)
        robot_l = robot_front_center - right * (self.width_px * 0.42)
        robot_r = robot_front_center + right * (self.width_px * 0.42)

        paddle_l = center - right * (half_w * 0.9)
        paddle_r = center + right * (half_w * 0.9)

        rod_l = (robot_l, paddle_l)
        rod_r = (robot_r, paddle_r)

        probe = center
        return rect_pts, rod_l, rod_r, probe

    def loader_paddle_loading_probe(self):
        if not self.loader_paddle_requested:
            return None
        if self.loader_paddle_fraction < ROBOT_PADDLE_LOADER_LOAD_FRACTION:
            return None
        geo = self.paddle_geometry(require_loaded=False)
        if geo is None:
            return None
        return geo[3]

    def _paddle_deployed_rect_points(self):
        geo = self.paddle_geometry(require_loaded=False)
        return [] if geo is None else geo[0]

    def _paddle_clear_to_deploy(self, space) -> bool:
        rect_pts = self._paddle_deployed_rect_points()
        if not rect_pts:
            return False

        margin = ROBOT_PADDLE_CLEARANCE_MARGIN_PX
        for point in rect_pts:
            if (
                point.x < margin
                or point.x > FIELD_SIZE_PX - margin
                or point.y < margin
                or point.y > FIELD_SIZE_PX - margin
            ):
                return False

        axes = _polygon_axes(rect_pts)
        for shape in getattr(space, "shapes", []):
            if shape is self.shape or getattr(shape, "is_field_floor", False):
                continue
            if getattr(shape, "is_goal_mouth_gate", False):
                continue
            if getattr(shape, "is_loader_ball", False):
                continue

            if getattr(shape, "is_block", False):
                if _circle_overlaps_poly(shape.body.position, getattr(shape, "block_pickup_radius_px", BLOCK_RADIUS_PX), rect_pts, axes):
                    return False
                continue

            if not getattr(shape.body, "dynamic", False) or getattr(shape, "is_loader_solid", False):
                other = [shape.body.local_to_world(v) for v in shape.get_vertices()]
                if other and _polygons_overlap(rect_pts, other, axes):
                    return False

        return True

    def _update_loader_paddle(self, dt: float, space):
        target = 1.0 if self.loader_paddle_requested else 0.0

        if target <= 0.0:
            self.loader_paddle_fraction = max(
                0.0,
                self.loader_paddle_fraction - (ROBOT_PADDLE_RETRACT_RATE * dt),
            )
        elif self._paddle_clear_to_deploy(space):
            self.loader_paddle_fraction = min(
                1.0,
                self.loader_paddle_fraction + (ROBOT_PADDLE_DEPLOY_RATE * dt),
            )
        else:
            self.loader_paddle_fraction = min(
                ROBOT_PADDLE_BLOCKED_FRACTION,
                self.loader_paddle_fraction + (ROBOT_PADDLE_DEPLOY_RATE * dt),
            )

        active = self.loader_paddle_fraction >= ROBOT_PADDLE_LOAD_FRACTION
        collision_active = self.loader_paddle_fraction >= ROBOT_PADDLE_COLLISION_FRACTION
        if active != self.loader_paddle_active:
            self.loader_paddle_active = active
        if collision_active != getattr(self.shape, "loader_paddle_collision_active", False):
            self._set_loader_paddle_collision_active(collision_active)

    # ---------------- drivetrain physics ----------------

    def _mixed_drive_commands(self, turn_scale: float = 1.0):
        turn = self.cmd_turn * self.drive.turn_input_gain * turn_scale
        left = self.cmd_forward + turn
        right = self.cmd_forward - turn

        max_mag = max(1.0, abs(left), abs(right))
        return left / max_mag, right / max_mag

    def _moving_turn_scale(self, forward_speed_mps: float) -> float:
        speed_ratio = clamp(
            abs(forward_speed_mps) / max(1e-6, self.drive.wheel_free_speed_mps),
            0.0,
            1.0,
        )
        return 1.0 - (self.drive.moving_turn_reduction * speed_ratio)

    def _moving_lateral_hold_scale(self, forward_speed_mps: float, turn_cmd: float) -> float:
        speed_ratio = clamp(
            abs(forward_speed_mps) / max(1e-6, self.drive.wheel_free_speed_mps),
            0.0,
            1.0,
        )
        turn_ratio = abs(turn_cmd)
        reduction = self.drive.moving_lateral_hold_reduction * speed_ratio * turn_ratio
        return clamp(1.0 - reduction, 0.08, 1.0)

    def _update_contact_response(self, dt: float, space):
        self.rebound_timer = max(0.0, self.rebound_timer - dt)
        self.parking_step_cooldown = max(0.0, self.parking_step_cooldown - dt)
        self.parking_contact_timer = max(0.0, self.parking_contact_timer - dt)

        if bullet is None:
            return

        robot_body_id = self.body.body_id
        contacts = bullet.getContactPoints(bodyA=robot_body_id, physicsClientId=space.client_id)
        if not contacts:
            return

        contact_data = {}
        for cp in contacts:
            other_id = cp[2] if cp[1] == robot_body_id else cp[1]
            other_shape = space.shape_for_body_id(other_id) if hasattr(space, "shape_for_body_id") else None
            if other_shape is None:
                continue

            normal_force_n = float(cp[9])
            impact_normal = Vec2(cp[7][0], cp[7][1])
            closing_speed_mps = pxps_to_mps(abs(self.body.velocity.dot(impact_normal)))
            cached = contact_data.get(other_id)
            if cached is None or normal_force_n > cached["normal_force_n"]:
                contact_data[other_id] = {
                    "shape": other_shape,
                    "normal_force_n": normal_force_n,
                    "impact_normal": impact_normal,
                    "closing_speed_mps": closing_speed_mps,
                }

        parking_contact_active = False
        parking_assist_applied = False
        block_contact_force_n = 0.0
        for contact in contact_data.values():
            other_shape = contact["shape"]
            normal_force_n = contact["normal_force_n"]
            impact_normal = contact["impact_normal"]
            closing_speed_mps = contact["closing_speed_mps"]
            if getattr(other_shape, "is_parking_zone_edge", False) and normal_force_n >= PARK_ZONE_STEP_FORCE_THRESHOLD_N:
                parking_contact_active = True
            if getattr(other_shape, "is_block", False):
                block_contact_force_n = max(block_contact_force_n, normal_force_n)

            if (
                normal_force_n >= ROBOT_REBOUND_FORCE_THRESHOLD_N
                and closing_speed_mps >= ROBOT_REBOUND_SPEED_THRESHOLD_MPS
                and (
                    getattr(other_shape, "is_field_wall", False)
                    or getattr(other_shape, "is_goal_surface", False)
                )
            ):
                if self.rebound_timer <= 0.0:
                    rebound_force_n = clamp(
                        ROBOT_REBOUND_IMPULSE_FORCE_N
                        + (normal_force_n * 0.05)
                        + (closing_speed_mps * 45.0),
                        0.0,
                        ROBOT_REBOUND_IMPULSE_FORCE_MAX_N,
                    )
                    bullet.applyExternalForce(
                        robot_body_id,
                        -1,
                        [impact_normal.x * rebound_force_n, impact_normal.y * rebound_force_n, 0.0],
                        [px_to_meters(self.body.position.x), px_to_meters(self.body.position.y), self.body.z],
                        bullet.WORLD_FRAME,
                        physicsClientId=space.client_id,
                    )
                self.rebound_timer = max(self.rebound_timer, ROBOT_REBOUND_TIME_SEC)

            if (
                PARK_ZONE_CONTACT_ASSIST_ENABLED
                and
                getattr(other_shape, "is_parking_zone_edge", False)
                and normal_force_n >= PARK_ZONE_STEP_FORCE_THRESHOLD_N
                and closing_speed_mps >= PARK_ZONE_STEP_SPEED_THRESHOLD_MPS
                and max(abs(self.body.roll), abs(self.body.pitch)) <= PARK_ZONE_MAX_TILT_RAD
            ):
                if parking_assist_applied:
                    continue
                outward = getattr(other_shape, "parking_zone_outward", None)
                if outward is None:
                    continue

                rel = self.body.position - other_shape.body.position
                inside_zone = rel.dot(outward) < 0.0
                outward_speed_mps = pxps_to_mps(self.body.velocity.dot(outward))
                drive_dir = self.front_vec() * self.cmd_forward
                drive_outward = drive_dir.dot(outward)

                wants_enter = (
                    (not inside_zone)
                    and drive_outward < -0.02
                    and abs(self.cmd_turn) <= PARK_ZONE_ENTER_ASSIST_TURN_LIMIT
                )
                wants_exit = inside_zone and drive_outward > 0.02 and abs(self.cmd_turn) <= PARK_ZONE_ENTER_ASSIST_TURN_LIMIT
                if not (wants_enter or wants_exit):
                    continue
                if self.parking_step_cooldown > 0.0:
                    continue

                tangent_dir = outward.perpendicular().normalized()
                tangent_speed_pxps = self.body.velocity.dot(tangent_dir)
                tangent_speed_cap_pxps = mps_to_pxps(PARK_ZONE_ENTER_TANGENT_SPEED_MAX_MPS)
                clamped_tangent_pxps = clamp(
                    tangent_speed_pxps,
                    -tangent_speed_cap_pxps,
                    tangent_speed_cap_pxps,
                )

                ramp_point = self.body.position - (outward * (self.length_px * 0.34))
                velocity_adjust = tangent_dir * (clamped_tangent_pxps - tangent_speed_pxps)

                if wants_enter:
                    lift_force_n = clamp(
                        PARK_ZONE_ENTER_LIFT_FORCE_N
                        + (normal_force_n * 0.10)
                        + (max(0.0, -drive_outward) * 22.0)
                        + (closing_speed_mps * 8.0),
                        0.0,
                        PARK_ZONE_ENTER_LIFT_FORCE_MAX_N,
                    )
                    bullet.applyExternalForce(
                        robot_body_id,
                        -1,
                        [0.0, 0.0, lift_force_n],
                        [
                            px_to_meters(ramp_point.x),
                            px_to_meters(ramp_point.y),
                            max(0.02, self.body.z - 0.11),
                        ],
                        bullet.WORLD_FRAME,
                        physicsClientId=space.client_id,
                    )
                    inward_force_n = clamp(
                        PARK_ZONE_ENTER_INWARD_PUSH_N
                        + (normal_force_n * 0.18)
                        + (max(0.0, -drive_outward) * 32.0),
                        0.0,
                        PARK_ZONE_ENTER_INWARD_PUSH_MAX_N,
                    )
                    bullet.applyExternalForce(
                        robot_body_id,
                        -1,
                        [(-outward.x) * inward_force_n, (-outward.y) * inward_force_n, 0.0],
                        [
                            px_to_meters(ramp_point.x),
                            px_to_meters(ramp_point.y),
                            max(0.02, self.body.z - 0.05),
                        ],
                        bullet.WORLD_FRAME,
                        physicsClientId=space.client_id,
                    )
                    inward_dir = -outward
                    current_inward_pxps = self.body.velocity.dot(inward_dir)
                    min_inward_pxps = mps_to_pxps(PARK_ZONE_ENTER_MIN_INWARD_SPEED_MPS)
                    max_inward_pxps = mps_to_pxps(PARK_ZONE_ENTER_MAX_INWARD_SPEED_MPS)
                    target_inward_pxps = clamp(current_inward_pxps, min_inward_pxps, max_inward_pxps)
                    velocity_adjust += inward_dir * (target_inward_pxps - current_inward_pxps)
                else:
                    exit_lift_force_n = clamp(
                        PARK_ZONE_EXIT_ASSIST_FORCE_N
                        + (normal_force_n * 0.08)
                        + (drive_outward * 18.0),
                        0.0,
                        PARK_ZONE_EXIT_ASSIST_FORCE_MAX_N,
                    )
                    bullet.applyExternalForce(
                        robot_body_id,
                        -1,
                        [0.0, 0.0, exit_lift_force_n],
                        [
                            px_to_meters(ramp_point.x),
                            px_to_meters(ramp_point.y),
                            max(0.02, self.body.z - 0.10),
                        ],
                        bullet.WORLD_FRAME,
                        physicsClientId=space.client_id,
                    )
                    bullet.applyExternalForce(
                        robot_body_id,
                        -1,
                        [outward.x * PARK_ZONE_EXIT_OUTWARD_PUSH_N, outward.y * PARK_ZONE_EXIT_OUTWARD_PUSH_N, 0.0],
                        [
                            px_to_meters(ramp_point.x),
                            px_to_meters(ramp_point.y),
                            max(0.02, self.body.z - 0.05),
                        ],
                        bullet.WORLD_FRAME,
                        physicsClientId=space.client_id,
                    )

                if velocity_adjust.length > 1e-4:
                    self.body.velocity = self.body.velocity + velocity_adjust
                self.body.angular_velocity *= PARK_ZONE_ENTER_YAW_DAMP_SCALE
                self.parking_step_cooldown = PARK_ZONE_STEP_COOLDOWN_SEC
                self.parking_contact_timer = max(self.parking_contact_timer, PARK_ZONE_CONTACT_STABILITY_TIME_SEC)
                parking_assist_applied = True

        if parking_contact_active:
            self.parking_contact_timer = PARK_ZONE_CONTACT_STABILITY_TIME_SEC

        if block_contact_force_n > 0.0 and self.parking_contact_timer <= 0.0:
            downforce_n = clamp(
                ROBOT_BLOCK_CONTACT_DOWNFORCE_N + (block_contact_force_n * 0.10),
                0.0,
                ROBOT_BLOCK_CONTACT_DOWNFORCE_MAX_N,
            )
            nose_point = self.body.position + (self.front_vec() * (self.length_px * 0.32))
            bullet.applyExternalForce(
                robot_body_id,
                -1,
                [0.0, 0.0, -downforce_n],
                [px_to_meters(nose_point.x), px_to_meters(nose_point.y), self.body.z],
                bullet.WORLD_FRAME,
                physicsClientId=space.client_id,
            )

        if self.parking_contact_timer > 0.0:
            speed_limit_pxps = mps_to_pxps(self.drive.wheel_free_speed_mps * PARK_ZONE_CONTACT_SPEED_LIMIT_SCALE)
            current_speed_pxps = self.body.velocity.length
            if current_speed_pxps > speed_limit_pxps and current_speed_pxps > 1e-6:
                self.body.velocity = self.body.velocity.normalized() * speed_limit_pxps
            if abs(self.body.angular_velocity) > PARK_ZONE_CONTACT_YAW_RATE_LIMIT_RAD_S:
                self.body.angular_velocity = clamp(
                    self.body.angular_velocity,
                    -PARK_ZONE_CONTACT_YAW_RATE_LIMIT_RAD_S,
                    PARK_ZONE_CONTACT_YAW_RATE_LIMIT_RAD_S,
                )

    def _apply_tilt_stabilization(self, space):
        if bullet is None:
            return

        ang_world = self.body.angular_velocity_world
        torque_x = clamp(
            (-(self.body.roll * ROBOT_TILT_STIFFNESS_NM)) - (ang_world[0] * ROBOT_TILT_DAMPING_NM),
            -ROBOT_TILT_TORQUE_MAX_NM,
            ROBOT_TILT_TORQUE_MAX_NM,
        )
        torque_y = clamp(
            (-(self.body.pitch * ROBOT_TILT_STIFFNESS_NM)) - (ang_world[1] * ROBOT_TILT_DAMPING_NM),
            -ROBOT_TILT_TORQUE_MAX_NM,
            ROBOT_TILT_TORQUE_MAX_NM,
        )
        if abs(torque_x) < 1e-4 and abs(torque_y) < 1e-4:
            return

        bullet.applyExternalTorque(
            self.body.body_id,
            -1,
            [torque_x, torque_y, 0.0],
            bullet.WORLD_FRAME,
            physicsClientId=space.client_id,
        )

    def _yaw_scrub_torque_nm(self, omega: float) -> float:
        if abs(omega) < 1e-4:
            return 0.0

        scrub_mag = (
            self.drive.yaw_scrub_coeff
            * self.body.mass
            * STD_GRAVITY
            * self.drive.track_width_m
        )
        return -_sign(omega) * scrub_mag

    def _side_motor_force_limit_n(self, throttle_mag: float, side_speed_mps: float) -> float:
        if throttle_mag < 1e-4:
            return 0.0

        cmd_free_speed = self.drive.wheel_free_speed_mps * throttle_mag
        if cmd_free_speed > 1e-5:
            speed_ratio = clamp(abs(side_speed_mps) / cmd_free_speed, 0.0, 1.0)
        else:
            speed_ratio = 0.0

        motor_cap = self.drive.side_stall_force_n * throttle_mag * (1.0 - speed_ratio)
        return min(motor_cap, self.drive.side_longitudinal_traction_n)

    def _side_drive_force_n(self, throttle: float, side_speed_mps: float, ground_scale: float) -> float:
        if ground_scale <= 1e-4:
            return 0.0

        throttle_mag = abs(throttle)
        side_mass = self.body.mass * 0.5
        target_speed = throttle * self.drive.wheel_free_speed_mps
        desired_accel = (target_speed - side_speed_mps) / self.drive.speed_controller_time_constant

        if throttle_mag < 1e-4:
            desired_force = side_mass * desired_accel
            cap_n = self.drive.neutral_brake_force_n * 0.5
        else:
            accel_cap = min(
                self.drive.side_stall_force_n,
                self.drive.side_longitudinal_traction_n,
            ) / max(1e-6, side_mass)
            brake_cap_accel = min(
                self.drive.side_stall_force_n * self.drive.active_brake_factor,
                self.drive.side_longitudinal_traction_n,
            ) / max(1e-6, side_mass)
            accel_limit = accel_cap if desired_accel * throttle >= 0.0 else brake_cap_accel
            desired_force = side_mass * clamp(desired_accel, -accel_limit, accel_limit)

            motor_cap = self._side_motor_force_limit_n(throttle_mag, side_speed_mps)
            brake_cap = min(
                self.drive.side_stall_force_n * self.drive.active_brake_factor * throttle_mag,
                self.drive.side_longitudinal_traction_n,
            )
            cap_n = motor_cap if desired_force * throttle >= 0.0 else brake_cap

        force_n = clamp(desired_force, -cap_n, cap_n)
        if abs(side_speed_mps) > 0.02:
            force_n += -_sign(side_speed_mps) * self.drive.side_rolling_resistance_n

        side_cap = self.drive.side_longitudinal_traction_n * ground_scale
        return clamp(force_n * ground_scale, -side_cap, side_cap)

    def _wheel_lateral_force_n(self, lateral_speed_mps: float, ground_scale: float, hold_scale: float) -> float:
        if ground_scale <= 1e-4 or abs(lateral_speed_mps) < 0.01:
            return 0.0

        desired_force = -(self.body.mass * 0.25) * (
            lateral_speed_mps / self.drive.lateral_time_constant
        )
        cap_n = self.drive.side_lateral_traction_n * 0.5 * ground_scale * hold_scale
        return clamp(desired_force, -cap_n, cap_n)

    def _wheel_ground_scale(self) -> float:
        tilt = max(abs(self.body.roll), abs(self.body.pitch))
        if tilt >= ROBOT_WHEEL_DRIVE_DISABLE_TILT_RAD:
            return 0.0
        if tilt <= ROBOT_WHEEL_DRIVE_FADE_TILT_RAD:
            tilt_scale = 1.0
        else:
            span = ROBOT_WHEEL_DRIVE_DISABLE_TILT_RAD - ROBOT_WHEEL_DRIVE_FADE_TILT_RAD
            tilt_scale = 1.0 - ((tilt - ROBOT_WHEEL_DRIVE_FADE_TILT_RAD) / max(1e-6, span))

        height_scale = clamp(
            (ROBOT_WHEEL_AIRBORNE_Z_M - self.body.z) / max(1e-6, ROBOT_WHEEL_AIRBORNE_Z_M - 0.18),
            0.0,
            1.0,
        )
        return clamp(tilt_scale * height_scale, 0.0, 1.0)

    def _wheel_local_points(self):
        half_track = min(self.drive.track_width_px * 0.5, self.width_px * 0.46)
        half_wheelbase = self.length_px * ROBOT_WHEELBASE_SCALE * 0.5
        return (
            ("left", Vec2(-half_track, -half_wheelbase)),
            ("left", Vec2(-half_track, half_wheelbase)),
            ("right", Vec2(half_track, -half_wheelbase)),
            ("right", Vec2(half_track, half_wheelbase)),
        )

    def _point_velocity_pxps(self, world_point: Vec2) -> Vec2:
        radius = world_point - self.body.position
        spin_velocity = Vec2(
            -self.body.angular_velocity * radius.y,
            self.body.angular_velocity * radius.x,
        )
        return self.body.velocity + spin_velocity

    def _apply_wheel_force_n(self, space, world_point: Vec2, force: Vec2):
        if bullet is None or force.length < 1e-6:
            return

        contact_z_m = max(0.008, min(ROBOT_WHEEL_CONTACT_Z_M, self.body.z - 0.040))
        bullet.applyExternalForce(
            self.body.body_id,
            -1,
            [force.x, force.y, 0.0],
            [
                px_to_meters(world_point.x),
                px_to_meters(world_point.y),
                contact_z_m,
            ],
            bullet.WORLD_FRAME,
            physicsClientId=space.client_id,
        )

    def _update_drivetrain_physics(self, dt: float, space):
        # Propulsion comes from wheel contact patches. Bullet then turns those
        # forces into linear motion, yaw, and any pitch/roll while climbing.
        self.body.force = (0.0, 0.0)
        self.body.torque = 0.0

        fwd = self.front_vec()
        right = self.right_vec()
        omega = self.body.angular_velocity
        body_velocity = self.body.velocity
        body_forward_mps = pxps_to_mps(body_velocity.dot(fwd))
        body_lateral_mps = pxps_to_mps(body_velocity.dot(right))

        half_track_m = self.drive.track_width_m * 0.5

        left_long_mps = body_forward_mps + (omega * half_track_m)
        right_long_mps = body_forward_mps - (omega * half_track_m)

        turn_scale = self._moving_turn_scale(body_forward_mps)
        lateral_hold_scale = self._moving_lateral_hold_scale(body_forward_mps, self.cmd_turn)
        if self.rebound_timer > 0.0:
            turn_scale *= 0.84
            lateral_hold_scale *= ROBOT_REBOUND_LATERAL_HOLD_SCALE
        if self.parking_contact_timer > 0.0:
            turn_scale *= PARK_ZONE_CONTACT_STABILITY_TURN_SCALE
            lateral_hold_scale = min(1.0, lateral_hold_scale * PARK_ZONE_CONTACT_STABILITY_HOLD_SCALE)

        left_cmd, right_cmd = self._mixed_drive_commands(turn_scale)
        ground_scale = self._wheel_ground_scale()
        if self.rebound_timer > 0.0 and abs(self.cmd_forward) < 1e-4 and abs(self.cmd_turn) < 1e-4:
            ground_scale *= 0.35

        left_force_n = self._side_drive_force_n(left_cmd, left_long_mps, ground_scale)
        right_force_n = self._side_drive_force_n(right_cmd, right_long_mps, ground_scale)
        side_force_n = {
            "left": left_force_n * 0.5,
            "right": right_force_n * 0.5,
        }

        lateral_force_total_n = 0.0
        for side, local_point in self._wheel_local_points():
            world_point = self.body.local_to_world(local_point)
            point_velocity = self._point_velocity_pxps(world_point)
            wheel_lateral_mps = pxps_to_mps(point_velocity.dot(right))
            lateral_force_n = self._wheel_lateral_force_n(
                wheel_lateral_mps,
                ground_scale,
                lateral_hold_scale,
            )
            lateral_force_total_n += lateral_force_n
            world_force = (fwd * side_force_n[side]) + (right * lateral_force_n)
            self._apply_wheel_force_n(space, world_point, world_force)

        target_omega = ((left_cmd - right_cmd) * self.drive.wheel_free_speed_mps) / max(
            self.drive.track_width_m,
            1e-6,
        )

        self.drive_debug = {
            "speed_mps": pxps_to_mps(body_velocity.length),
            "forward_speed_mps": body_forward_mps,
            "lateral_speed_mps": body_lateral_mps,
            "yaw_rate_deg_s": math.degrees(omega),
            "roll_deg": math.degrees(self.body.roll),
            "pitch_deg": math.degrees(self.body.pitch),
            "left_cmd": left_cmd,
            "right_cmd": right_cmd,
            "left_speed_mps": left_long_mps,
            "right_speed_mps": right_long_mps,
            "drive_force_n": left_force_n + right_force_n,
            "lateral_force_n": lateral_force_total_n,
            "turn_scale": turn_scale,
            "lateral_hold_scale": lateral_hold_scale,
            "target_yaw_rate_deg_s": math.degrees(target_omega),
            "wheel_ground_scale": ground_scale,
            "dt": dt,
        }

    # ---------------- update ----------------

    def update(self, dt: float, space, goals, loaders):
        self._update_contact_response(dt, space)
        self._update_drivetrain_physics(dt, space)
        self._apply_tilt_stabilization(space)
        self._update_loader_paddle(dt, space)
        self._update_pickup(dt)
        self.try_intake_from_field(space)
        loaders.try_load_into_robot(self, space)
        self._update_dump(dt, space, goals)
