import atexit
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

try:
    import pybullet as bullet
except ImportError as exc:  # pragma: no cover - exercised only without dependency
    bullet = None
    _PYBULLET_IMPORT_ERROR = exc
else:
    _PYBULLET_IMPORT_ERROR = None

from config import (
    FIELD_SIZE_PX,
    FIXED_DT,
    PX_PER_M,
    STD_GRAVITY,
    px_to_meters,
    meters_to_px,
    BLOCK_MASS,
    BLOCK_RADIUS_PX,
    BLOCK_SURFACE_FRICTION,
    BLOCK_STOP_SPEED_PX,
    BLOCK_STOP_ANG_VEL,
    AIR_TIME_SEC,
    INITIAL_BLOCKS_LAYOUT,
    inches_to_px,
    inches_to_m,
    LOADER_PADDLE_LEN_PX,
    LOADER_PADDLE_W_PX,
    LOADER_PADDLE_T_PX,
    PARK_ZONE_OUTER_SIZE_IN,
    PARK_ZONE_INNER_SIZE_IN,
    PARK_ZONE_WALL_THICKNESS_IN,
    PARK_ZONE_WALL_HEIGHT_IN,
)
from triball_mesh import load_triball_mesh
from vec2 import Vec2, as_vec2


ASSETS_DIR = Path(__file__).resolve().parent / "assets"
TRIBALL_ASSET_PATH = ASSETS_DIR / "vex-v5-push-back-hex-ball.stl"
PARK_ZONE_BAR_WEDGE_ASSET_PATH = ASSETS_DIR / "parking-zone-bar-wedge.obj"
TRIBALL_MESH = load_triball_mesh(TRIBALL_ASSET_PATH)

TRIBALL_VERTICES = list(TRIBALL_MESH.render_footprint_px)

ROBOT_BODY_HEIGHT_M = 0.28
ROBOT_COM_OFFSET_DOWN_M = 0.05
ROBOT_CHASSIS_HEIGHT_M = 0.22
ROBOT_CHASSIS_CLEARANCE_M = 0.014
ROBOT_CHASSIS_FOOTPRINT_SCALE = 0.82
ROBOT_WHEEL_EDGE_MARGIN_SCALE = 0.10
ROBOT_WHEEL_FRONT_REAR_MARGIN_SCALE = 0.02
ROBOT_WHEEL_CONTACT_FRICTION = 0.08
ROBOT_CHASSIS_CONTACT_FRICTION_SCALE = 0.58
ROBOT_PADDLE_HEIGHT_M = 0.055
ROBOT_PADDLE_GROUND_CLEARANCE_M = 0.012
ROBOT_CHASSIS_COLLISION_GROUP = 1
PARK_ZONE_COLLISION_GROUP = 2
ROBOT_WHEEL_COLLISION_GROUP = 4
BLOCK_COLLISION_GROUP = 8
ROBOT_BUMPER_COLLISION_GROUP = 16
ROBOT_NON_PARKING_MASK = 0xFFFF
ROBOT_WHEEL_MASK = PARK_ZONE_COLLISION_GROUP
ROBOT_CHASSIS_MASK = 0xFFFF
ROBOT_BUMPER_MASK = BLOCK_COLLISION_GROUP
BLOCK_COLLISION_MASK = 0xFFFF
ROBOT_BUMPER_THICKNESS_PX = BLOCK_RADIUS_PX * 0.70
ROBOT_BUMPER_FRICTION = 0.36
ROBOT_BUMPER_RESTITUTION = 0.02
BLOCK_BODY_HEIGHT_M = TRIBALL_MESH.height_m
ROBOT_BUMPER_HEIGHT_M = BLOCK_BODY_HEIGHT_M * 0.96
STATIC_OBSTACLE_HEIGHT_M = 0.16
FIELD_WALL_HEIGHT_M = 0.16
FLOOR_THICKNESS_M = 0.02
FIELD_WALL_THICKNESS_PX = 18.0
PARK_ZONE_OUTER_SIZE_PX = inches_to_px(PARK_ZONE_OUTER_SIZE_IN)
PARK_ZONE_INNER_SIZE_PX = inches_to_px(PARK_ZONE_INNER_SIZE_IN)
PARK_ZONE_WALL_THICKNESS_PX = inches_to_px(PARK_ZONE_WALL_THICKNESS_IN)
PARK_ZONE_WALL_HEIGHT_M = inches_to_m(PARK_ZONE_WALL_HEIGHT_IN)
PARK_ZONE_COLLISION_HEIGHT_SCALE = 0.34
PARK_ZONE_COLLISION_HEIGHT_M = PARK_ZONE_WALL_HEIGHT_M * PARK_ZONE_COLLISION_HEIGHT_SCALE
PARK_ZONE_CONTACT_FRICTION = 0.28
PARK_ZONE_CONTACT_RESTITUTION = 0.01

DEFAULT_BOX_VISUAL_RGBA = (0.56, 0.58, 0.60, 1.0)
DEFAULT_WEDGE_VISUAL_RGBA = (0.72, 0.25, 0.22, 1.0)
TRIBALL_RED_VISUAL_RGBA = (1.0, 0.18, 0.12, 1.0)
TRIBALL_BLUE_VISUAL_RGBA = (0.10, 0.30, 1.0, 1.0)
ROBOT_CHASSIS_VISUAL_RGBA = (0.70, 0.74, 0.78, 1.0)
ROBOT_BUMPER_VISUAL_RGBA = (0.16, 0.18, 0.22, 1.0)
ROBOT_PADDLE_VISUAL_RGBA = (1.0, 0.84, 0.36, 1.0)
ROBOT_ROD_VISUAL_RGBA = (0.82, 0.84, 0.86, 1.0)

PYBULLET_SOLVER_ITERS = 28
PYBULLET_SOLVER_ITERS_MEDIUM_LOAD = 22
PYBULLET_SOLVER_ITERS_HIGH_LOAD = 18
PYBULLET_SOLVER_MEDIUM_BLOCK_COUNT = 24
PYBULLET_SOLVER_HIGH_BLOCK_COUNT = 36
PYBULLET_CONTACT_BREAKING_THRESHOLD_M = 0.008
PYBULLET_RESTITUTION_VELOCITY_THRESHOLD = 0.03

FIELD_FLOOR_FRICTION = 0.48
FIELD_FLOOR_RESTITUTION = 0.0
FIELD_WALL_RESTITUTION = 0.20

BLOCK_CONTACT_FRICTION = 2.94
BLOCK_CONTACT_ROLLING_FRICTION = 0.00134
BLOCK_CONTACT_SPINNING_FRICTION = 0.00168
BLOCK_CONTACT_RESTITUTION = 0.08

ROBOT_CONTACT_FRICTION = 0.28
ROBOT_CONTACT_ROLLING_FRICTION = 0.0001
ROBOT_CONTACT_SPINNING_FRICTION = 0.0010
ROBOT_CONTACT_RESTITUTION = 0.32

BLOCK_SLEEP_SPEED_PX = 0.45
BLOCK_SLEEP_YAW_RATE = 0.05
BLOCK_SLEEP_ANG_WORLD = 0.12
BLOCK_SLEEP_LINEAR_Z = 0.03
BLOCK_SLEEP_TIME = 0.18
BLOCK_ROLL_ASSIST_GAIN = 14.0
BLOCK_ROLL_ASSIST_MIN_SPEED_PX = 1.5
BLOCK_ROLL_ASSIST_ANG_ERROR_MIN = 0.85
BLOCK_EFFECTIVE_ROLL_RADIUS_M = 0.5 * (TRIBALL_MESH.width_m + TRIBALL_MESH.depth_m) * 0.5
BLOCK_CONTAINMENT_MARGIN_PX = BLOCK_RADIUS_PX * 0.95
BLOCK_MIN_CENTER_Z_M = BLOCK_BODY_HEIGHT_M * 0.30
BLOCK_RESCUE_CENTER_Z_M = BLOCK_BODY_HEIGHT_M * 0.52
BLOCK_MAX_CENTER_Z_M = 1.0
BLOCK_MAX_SPEED_PXPS = 1650.0
BLOCK_MAX_VERTICAL_SPEED_MPS = 4.0


def _require_pybullet():
    if bullet is None:
        raise RuntimeError(
            "PyBullet is required for the physics engine.\n"
            f"Current interpreter: {sys.executable}\n"
            f"Python version: {sys.version.split()[0]}\n"
            "Install it into this exact interpreter with:\n"
            f"`\"{sys.executable}\" -m pip install pybullet`"
        ) from _PYBULLET_IMPORT_ERROR


def _yaw_to_quat(yaw_rad: float):
    return bullet.getQuaternionFromEuler((0.0, 0.0, yaw_rad))


def _quat_to_yaw(quat) -> float:
    return bullet.getEulerFromQuaternion(quat)[2]


def _inertia_si_to_sim(inertia_kg_m2: float) -> float:
    return inertia_kg_m2 * (PX_PER_M ** 2)


@dataclass
class PhysicsShape:
    body: "PhysicsBody"
    kind: str
    local_vertices: List[Vec2] = field(default_factory=list)
    radius: float = 0.0
    friction: float = 0.0
    elasticity: float = 0.0

    def get_vertices(self) -> List[Vec2]:
        return list(self.local_vertices)


class PhysicsBody:
    def __init__(
        self,
        world: "PhysicsWorld",
        body_id: int,
        *,
        mass: float,
        moment: float,
        z_m: float,
        dynamic: bool,
        planar_lock: bool,
        gravity_scale: float,
    ):
        self.world = world
        self.body_id = body_id
        self.mass = float(mass)
        self.moment = float(moment)
        self.lock_z_m = float(z_m)
        self.dynamic = bool(dynamic)
        self.planar_lock = bool(planar_lock)
        self.gravity_scale = float(gravity_scale)

        self._position = Vec2(0.0, 0.0)
        self._angle = 0.0
        self._velocity = Vec2(0.0, 0.0)
        self._angular_velocity = 0.0
        self._force = Vec2(0.0, 0.0)
        self._torque = 0.0

        self._z_m = float(z_m)
        self._quat = (0.0, 0.0, 0.0, 1.0)
        self._roll = 0.0
        self._pitch = 0.0
        self._linear_velocity_z = 0.0
        self._angular_velocity_world = (0.0, 0.0, 0.0)

    @property
    def position(self) -> Vec2:
        return self._position

    @position.setter
    def position(self, value):
        self._position = as_vec2(value)
        quat = _yaw_to_quat(self._angle) if self.planar_lock else self._quat
        z = self.lock_z_m if self.planar_lock else self._z_m
        if self.world.connected:
            bullet.resetBasePositionAndOrientation(
                self.body_id,
                [
                    px_to_meters(self._position.x),
                    px_to_meters(self._position.y),
                    z,
                ],
                quat,
                physicsClientId=self.world.client_id,
            )

    @property
    def angle(self) -> float:
        return self._angle

    @angle.setter
    def angle(self, value: float):
        self._angle = float(value)
        quat = _yaw_to_quat(self._angle) if self.planar_lock else bullet.getQuaternionFromEuler(
            (self._roll, self._pitch, self._angle)
        )
        z = self.lock_z_m if self.planar_lock else self._z_m
        if self.world.connected:
            bullet.resetBasePositionAndOrientation(
                self.body_id,
                [
                    px_to_meters(self._position.x),
                    px_to_meters(self._position.y),
                    z,
                ],
                quat,
                physicsClientId=self.world.client_id,
            )

    @property
    def velocity(self) -> Vec2:
        return self._velocity

    @velocity.setter
    def velocity(self, value):
        self._velocity = as_vec2(value)
        lin_z = 0.0 if self.planar_lock else self._linear_velocity_z
        ang = [0.0, 0.0, self._angular_velocity] if self.planar_lock else [
            self._angular_velocity_world[0],
            self._angular_velocity_world[1],
            self._angular_velocity,
        ]
        if self.world.connected:
            bullet.resetBaseVelocity(
                self.body_id,
                linearVelocity=[
                    self._velocity.x / PX_PER_M,
                    self._velocity.y / PX_PER_M,
                    lin_z,
                ],
                angularVelocity=ang,
                physicsClientId=self.world.client_id,
            )

    @property
    def angular_velocity(self) -> float:
        return self._angular_velocity

    @angular_velocity.setter
    def angular_velocity(self, value: float):
        self._angular_velocity = float(value)
        lin_z = 0.0 if self.planar_lock else self._linear_velocity_z
        ang = [0.0, 0.0, self._angular_velocity] if self.planar_lock else [
            self._angular_velocity_world[0],
            self._angular_velocity_world[1],
            self._angular_velocity,
        ]
        if self.world.connected:
            bullet.resetBaseVelocity(
                self.body_id,
                linearVelocity=[
                    self._velocity.x / PX_PER_M,
                    self._velocity.y / PX_PER_M,
                    lin_z,
                ],
                angularVelocity=ang,
                physicsClientId=self.world.client_id,
            )

    @property
    def force(self) -> Vec2:
        return self._force

    @force.setter
    def force(self, value):
        self._force = as_vec2(value)

    @property
    def torque(self) -> float:
        return self._torque

    @torque.setter
    def torque(self, value: float):
        self._torque = float(value)

    @property
    def z(self) -> float:
        return self._z_m

    @property
    def roll(self) -> float:
        return self._roll

    @property
    def pitch(self) -> float:
        return self._pitch

    @property
    def quat(self):
        return self._quat

    @property
    def angular_velocity_world(self):
        return self._angular_velocity_world

    def apply_force_at_world_point(self, force, point):
        force_v = as_vec2(force)
        point_v = as_vec2(point)
        self._force = self._force + force_v
        arm = point_v - self._position
        self._torque += arm.cross(force_v)

    def local_to_world(self, point) -> Vec2:
        point_v = as_vec2(point)
        return self._position + point_v.rotated(self._angle)

    def sync_from_bullet(self):
        pos, quat = bullet.getBasePositionAndOrientation(
            self.body_id,
            physicsClientId=self.world.client_id,
        )
        lin_vel, ang_vel = bullet.getBaseVelocity(
            self.body_id,
            physicsClientId=self.world.client_id,
        )

        roll, pitch, yaw = bullet.getEulerFromQuaternion(quat)

        if self.planar_lock:
            needs_planar_fix = (
                abs(pos[2] - self.lock_z_m) > 1e-5
                or abs(roll) > 1e-4
                or abs(pitch) > 1e-4
                or abs(lin_vel[2]) > 1e-5
                or abs(ang_vel[0]) > 1e-4
                or abs(ang_vel[1]) > 1e-4
            )
            if needs_planar_fix:
                bullet.resetBasePositionAndOrientation(
                    self.body_id,
                    [pos[0], pos[1], self.lock_z_m],
                    _yaw_to_quat(yaw),
                    physicsClientId=self.world.client_id,
                )
                bullet.resetBaseVelocity(
                    self.body_id,
                    linearVelocity=[lin_vel[0], lin_vel[1], 0.0],
                    angularVelocity=[0.0, 0.0, ang_vel[2]],
                    physicsClientId=self.world.client_id,
                )
                pos = [pos[0], pos[1], self.lock_z_m]
                quat = _yaw_to_quat(yaw)
                roll = 0.0
                pitch = 0.0
                lin_vel = [lin_vel[0], lin_vel[1], 0.0]
                ang_vel = [0.0, 0.0, ang_vel[2]]

        self._position = Vec2(meters_to_px(pos[0]), meters_to_px(pos[1]))
        self._z_m = pos[2]
        self._quat = quat
        self._roll = roll
        self._pitch = pitch
        self._angle = yaw
        self._velocity = Vec2(meters_to_px(lin_vel[0]), meters_to_px(lin_vel[1]))
        self._linear_velocity_z = lin_vel[2]
        self._angular_velocity = ang_vel[2]
        self._angular_velocity_world = tuple(ang_vel)

    def clear_external_loads(self):
        self._force = Vec2(0.0, 0.0)
        self._torque = 0.0


class PhysicsWorld:
    def __init__(self):
        _require_pybullet()

        self.client_id = bullet.connect(bullet.DIRECT)
        self.connected = self.client_id >= 0
        self._closed = False

        self.shapes: List[PhysicsShape] = []
        self.bodies: List[PhysicsBody] = []
        self.block_shapes: List[PhysicsShape] = []
        self._uid_to_body: Dict[int, PhysicsBody] = {}
        self._uid_to_shape: Dict[int, PhysicsShape] = {}
        self._box_shape_cache: Dict[tuple[float, float, float], int] = {}
        self._box_visual_cache: Dict[tuple[float, ...], int] = {}
        self._sphere_shape_cache: Dict[float, int] = {}
        self._mesh_shape_cache: Dict[tuple[str, float, float, float], int] = {}
        self._mesh_visual_cache: Dict[tuple[str, float, float, float, float, float, float, float], int] = {}
        self._triball_visual_cache: Dict[tuple[float, float, float, float], int] = {}
        self._triball_shape_id: Optional[int] = None
        self._dynamic_bodies: List[PhysicsBody] = []
        self.field_feature_shapes: List[PhysicsShape] = []
        self.fixed_dt = FIXED_DT
        self._solver_iterations = self._target_solver_iterations()

        bullet.setGravity(0.0, 0.0, -STD_GRAVITY, physicsClientId=self.client_id)
        bullet.setPhysicsEngineParameter(
            fixedTimeStep=self.fixed_dt,
            numSolverIterations=self._solver_iterations,
            numSubSteps=0,
            useSplitImpulse=1,
            splitImpulsePenetrationThreshold=-0.01,
            contactBreakingThreshold=PYBULLET_CONTACT_BREAKING_THRESHOLD_M,
            restitutionVelocityThreshold=PYBULLET_RESTITUTION_VELOCITY_THRESHOLD,
            physicsClientId=self.client_id,
        )
        atexit.register(self.close)

    def close(self):
        if self._closed:
            return
        if self.connected:
            bullet.disconnect(physicsClientId=self.client_id)
        self._closed = True
        self.connected = False

    def _cached_box_shape(self, half_extents_m) -> int:
        key = tuple(round(float(v), 6) for v in half_extents_m)
        shape_id = self._box_shape_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createCollisionShape(
                bullet.GEOM_BOX,
                halfExtents=list(key),
                physicsClientId=self.client_id,
            )
            self._box_shape_cache[key] = shape_id
        return shape_id

    def _cached_box_visual(self, half_extents_m, rgba=DEFAULT_BOX_VISUAL_RGBA) -> int:
        if rgba is None:
            return -1

        dims = tuple(round(float(v), 6) for v in half_extents_m)
        color = tuple(round(float(v), 4) for v in rgba)
        key = (*dims, *color)
        shape_id = self._box_visual_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createVisualShape(
                bullet.GEOM_BOX,
                halfExtents=list(dims),
                rgbaColor=list(color),
                specularColor=[0.22, 0.22, 0.22],
                physicsClientId=self.client_id,
            )
            self._box_visual_cache[key] = shape_id
        return shape_id

    def _triball_collision_shape(self) -> int:
        if self._triball_shape_id is None:
            self._triball_shape_id = bullet.createCollisionShape(
                bullet.GEOM_MESH,
                fileName=str(TRIBALL_MESH.collision_obj_path),
                physicsClientId=self.client_id,
            )
        return self._triball_shape_id

    def _triball_visual_shape(self, color: str) -> int:
        rgba = TRIBALL_BLUE_VISUAL_RGBA if color == "blue" else TRIBALL_RED_VISUAL_RGBA
        key = tuple(round(float(v), 4) for v in rgba)
        shape_id = self._triball_visual_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createVisualShape(
                bullet.GEOM_MESH,
                fileName=str(TRIBALL_MESH.collision_obj_path),
                rgbaColor=list(key),
                specularColor=[0.34, 0.30, 0.24],
                physicsClientId=self.client_id,
            )
            self._triball_visual_cache[key] = shape_id
        return shape_id

    def _cached_sphere_shape(self, radius_m: float) -> int:
        key = round(float(radius_m), 6)
        shape_id = self._sphere_shape_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createCollisionShape(
                bullet.GEOM_SPHERE,
                radius=key,
                physicsClientId=self.client_id,
            )
            self._sphere_shape_cache[key] = shape_id
        return shape_id

    def _cached_mesh_shape(self, mesh_path: Path, mesh_scale) -> int:
        key = (
            str(mesh_path),
            round(float(mesh_scale[0]), 6),
            round(float(mesh_scale[1]), 6),
            round(float(mesh_scale[2]), 6),
        )
        shape_id = self._mesh_shape_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createCollisionShape(
                bullet.GEOM_MESH,
                fileName=str(mesh_path),
                meshScale=[key[1], key[2], key[3]],
                physicsClientId=self.client_id,
            )
            self._mesh_shape_cache[key] = shape_id
        return shape_id

    def _cached_mesh_visual(self, mesh_path: Path, mesh_scale, rgba=DEFAULT_WEDGE_VISUAL_RGBA) -> int:
        color = tuple(round(float(v), 4) for v in rgba)
        key = (
            str(mesh_path),
            round(float(mesh_scale[0]), 6),
            round(float(mesh_scale[1]), 6),
            round(float(mesh_scale[2]), 6),
            *color,
        )
        shape_id = self._mesh_visual_cache.get(key)
        if shape_id is None:
            shape_id = bullet.createVisualShape(
                bullet.GEOM_MESH,
                fileName=str(mesh_path),
                meshScale=[key[1], key[2], key[3]],
                rgbaColor=list(color),
                specularColor=[0.20, 0.20, 0.20],
                physicsClientId=self.client_id,
            )
            self._mesh_visual_cache[key] = shape_id
        return shape_id

    def _register_body_and_shape(self, body: PhysicsBody, shape: PhysicsShape):
        self.bodies.append(body)
        self.shapes.append(shape)
        self._uid_to_body[body.body_id] = body
        self._uid_to_shape[body.body_id] = shape
        if body.dynamic:
            self._dynamic_bodies.append(body)
        body.sync_from_bullet()

    def shape_for_body_id(self, body_id: int) -> Optional[PhysicsShape]:
        return self._uid_to_shape.get(body_id)

    def _create_box_body(
        self,
        center_px,
        size_px,
        *,
        mass_kg: float,
        angle: float = 0.0,
        friction: float = 1.0,
        restitution: float = 0.0,
        rolling_friction: float = 0.0,
        spinning_friction: float = 0.0,
        height_m: float = STATIC_OBSTACLE_HEIGHT_M,
        z_center_m: Optional[float] = None,
        planar_lock: bool = True,
        gravity_scale: float = 0.0,
        visual_rgba=DEFAULT_BOX_VISUAL_RGBA,
    ) -> PhysicsShape:
        center_v = as_vec2(center_px)
        size_v = as_vec2(size_px)
        half_extents_m = [
            px_to_meters(size_v.x * 0.5),
            px_to_meters(size_v.y * 0.5),
            height_m * 0.5,
        ]
        collision_shape = self._cached_box_shape(half_extents_m)
        visual_shape = self._cached_box_visual(half_extents_m, visual_rgba)
        if z_center_m is None:
            z_center_m = height_m * 0.5

        body_id = bullet.createMultiBody(
            baseMass=float(mass_kg),
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[
                px_to_meters(center_v.x),
                px_to_meters(center_v.y),
                z_center_m,
            ],
            baseOrientation=_yaw_to_quat(angle),
            physicsClientId=self.client_id,
        )
        bullet.changeDynamics(
            body_id,
            -1,
            lateralFriction=float(friction),
            rollingFriction=float(rolling_friction),
            spinningFriction=float(spinning_friction),
            restitution=float(restitution),
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )

        half_w = size_v.x * 0.5
        half_h = size_v.y * 0.5
        verts = [
            Vec2(-half_w, -half_h),
            Vec2(half_w, -half_h),
            Vec2(half_w, half_h),
            Vec2(-half_w, half_h),
        ]
        moment_z = math.inf
        if mass_kg > 0.0:
            width_m = px_to_meters(size_v.x)
            depth_m = px_to_meters(size_v.y)
            moment_z_si = (mass_kg * ((width_m * width_m) + (depth_m * depth_m))) / 12.0
            moment_z = _inertia_si_to_sim(moment_z_si)

        body = PhysicsBody(
            self,
            body_id,
            mass=mass_kg,
            moment=moment_z,
            z_m=z_center_m,
            dynamic=mass_kg > 0.0,
            planar_lock=planar_lock,
            gravity_scale=gravity_scale,
        )
        shape = PhysicsShape(
            body=body,
            kind="poly",
            local_vertices=verts,
            friction=float(friction),
            elasticity=float(restitution),
        )
        shape.render_kind = "box"
        shape.render_size_px = (size_v.x, size_v.y)
        shape.render_height_m = float(height_m)
        shape.render_rgba = visual_rgba
        self._register_body_and_shape(body, shape)
        return shape

    def create_static_box(
        self,
        center_px,
        size_px,
        *,
        angle: float = 0.0,
        friction: float = 1.0,
        restitution: float = 0.0,
        rolling_friction: float = 0.0,
        spinning_friction: float = 0.0,
        height_m: float = STATIC_OBSTACLE_HEIGHT_M,
        z_center_m: Optional[float] = None,
        visual_rgba=DEFAULT_BOX_VISUAL_RGBA,
    ) -> PhysicsShape:
        return self._create_box_body(
            center_px,
            size_px,
            mass_kg=0.0,
            angle=angle,
            friction=friction,
            restitution=restitution,
            rolling_friction=rolling_friction,
            spinning_friction=spinning_friction,
            height_m=height_m,
            z_center_m=z_center_m,
            planar_lock=True,
            gravity_scale=0.0,
            visual_rgba=visual_rgba,
        )

    def create_static_sphere(
        self,
        center_px,
        radius_px: float,
        *,
        z_center_m: float,
        friction: float = 0.8,
        restitution: float = 0.02,
        visual_rgba=None,
    ) -> PhysicsShape:
        center_v = as_vec2(center_px)
        radius_m = px_to_meters(radius_px)
        collision_shape = self._cached_sphere_shape(radius_m)
        visual_shape = -1
        if visual_rgba is not None:
            visual_shape = bullet.createVisualShape(
                bullet.GEOM_SPHERE,
                radius=radius_m,
                rgbaColor=list(visual_rgba),
                specularColor=[0.18, 0.18, 0.18],
                physicsClientId=self.client_id,
            )

        body_id = bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[
                px_to_meters(center_v.x),
                px_to_meters(center_v.y),
                z_center_m,
            ],
            physicsClientId=self.client_id,
        )
        bullet.changeDynamics(
            body_id,
            -1,
            lateralFriction=float(friction),
            restitution=float(restitution),
            rollingFriction=0.0,
            spinningFriction=0.0,
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )

        verts = []
        sides = 12
        for i in range(sides):
            theta = math.tau * i / sides
            verts.append(Vec2(math.cos(theta) * radius_px, math.sin(theta) * radius_px))

        body = PhysicsBody(
            self,
            body_id,
            mass=0.0,
            moment=math.inf,
            z_m=z_center_m,
            dynamic=False,
            planar_lock=True,
            gravity_scale=0.0,
        )
        shape = PhysicsShape(
            body=body,
            kind="circle",
            local_vertices=verts,
            radius=float(radius_px),
            friction=float(friction),
            elasticity=float(restitution),
        )
        shape.render_rgba = visual_rgba
        shape.render_kind = "sphere"
        self._register_body_and_shape(body, shape)
        return shape

    def create_robot_box(
        self,
        center_px,
        size_px,
        *,
        mass_kg: float,
        angle: float = 0.0,
        friction: float = ROBOT_CONTACT_FRICTION,
        restitution: float = ROBOT_CONTACT_RESTITUTION,
        wheel_radius_m: float,
        track_width_m: float,
    ) -> PhysicsShape:
        center_v = as_vec2(center_px)
        size_v = as_vec2(size_px)

        width_m = px_to_meters(size_v.x)
        depth_m = px_to_meters(size_v.y)
        chassis_half_extents_m = [
            width_m * 0.5,
            depth_m * 0.5,
            ROBOT_BODY_HEIGHT_M * 0.5,
        ]
        chassis_collision_shape = self._cached_box_shape(chassis_half_extents_m)
        chassis_visual_shape = self._cached_box_visual(chassis_half_extents_m, ROBOT_CHASSIS_VISUAL_RGBA)
        z_center_m = ROBOT_BODY_HEIGHT_M * 0.5
        identity_quat = (0.0, 0.0, 0.0, 1.0)

        paddle_contact_z_m = max(
            (ROBOT_PADDLE_HEIGHT_M * 0.5) + ROBOT_PADDLE_GROUND_CLEARANCE_M,
            ROBOT_PADDLE_HEIGHT_M * 0.5,
        )
        paddle_local_z_m = paddle_contact_z_m - z_center_m
        paddle_center_y_m = -px_to_meters((size_v.y * 0.5) + (LOADER_PADDLE_LEN_PX * 0.55))
        paddle_half_extents_m = [
            px_to_meters(LOADER_PADDLE_W_PX * 0.5),
            px_to_meters(LOADER_PADDLE_T_PX * 0.5),
            ROBOT_PADDLE_HEIGHT_M * 0.5,
        ]
        paddle_collision_shape = self._cached_box_shape(paddle_half_extents_m)
        paddle_visual_shape = self._cached_box_visual(paddle_half_extents_m, ROBOT_PADDLE_VISUAL_RGBA)
        bumper_half_height_m = ROBOT_BUMPER_HEIGHT_M * 0.5
        bumper_center_z_m = bumper_half_height_m - z_center_m
        bumper_thickness_m = px_to_meters(ROBOT_BUMPER_THICKNESS_PX)
        front_blocker_half_extents_m = [
            width_m * 0.46,
            bumper_thickness_m * 0.5,
            bumper_half_height_m,
        ]
        front_blocker_shape = self._cached_box_shape(front_blocker_half_extents_m)
        front_blocker_visual_shape = self._cached_box_visual(
            front_blocker_half_extents_m,
            ROBOT_BUMPER_VISUAL_RGBA,
        )
        front_blocker_y_m = -(depth_m * 0.5) + (bumper_thickness_m * 0.5)

        rod_thickness_m = px_to_meters(max(3.0, LOADER_PADDLE_T_PX * 0.42))
        rod_height_m = ROBOT_PADDLE_HEIGHT_M * 0.72
        paddle_half_w_m = px_to_meters(LOADER_PADDLE_W_PX * 0.5 * 0.9)
        front_anchor_y_m = -px_to_meters(size_v.y * 0.5)
        front_anchor_x_m = px_to_meters(size_v.x * 0.42)
        rod_left_start = Vec2(-front_anchor_x_m, front_anchor_y_m)
        rod_left_end = Vec2(-paddle_half_w_m, paddle_center_y_m)
        rod_delta = rod_left_end - rod_left_start
        rod_length_m = max(0.01, rod_delta.length)
        rod_center_y_m = 0.5 * (rod_left_start.y + rod_left_end.y)
        rod_center_x_m = 0.5 * (rod_left_start.x + rod_left_end.x)
        rod_yaw = math.atan2(rod_delta.y, rod_delta.x)
        rod_half_extents_m = [
            rod_length_m * 0.5,
            rod_thickness_m * 0.5,
            rod_height_m * 0.5,
        ]
        rod_collision_shape = self._cached_box_shape(rod_half_extents_m)
        rod_visual_shape = self._cached_box_visual(rod_half_extents_m, ROBOT_ROD_VISUAL_RGBA)

        link_collision_shapes = [
            front_blocker_shape,
            rod_collision_shape,
            rod_collision_shape,
            paddle_collision_shape,
        ]
        link_visual_shapes = [
            front_blocker_visual_shape,
            rod_visual_shape,
            rod_visual_shape,
            paddle_visual_shape,
        ]
        link_positions = [
            [0.0, front_blocker_y_m, bumper_center_z_m],
            [rod_center_x_m, rod_center_y_m, paddle_local_z_m],
            [-rod_center_x_m, rod_center_y_m, paddle_local_z_m],
            [0.0, paddle_center_y_m, paddle_local_z_m],
        ]
        link_orientations = [
            identity_quat,
            _yaw_to_quat(rod_yaw),
            _yaw_to_quat(math.pi - rod_yaw),
            identity_quat,
        ]
        body_id = bullet.createMultiBody(
            baseMass=float(mass_kg),
            baseCollisionShapeIndex=chassis_collision_shape,
            baseVisualShapeIndex=chassis_visual_shape,
            basePosition=[
                px_to_meters(center_v.x),
                px_to_meters(center_v.y),
                z_center_m,
            ],
            baseOrientation=_yaw_to_quat(angle),
            baseInertialFramePosition=[0.0, 0.0, -ROBOT_COM_OFFSET_DOWN_M],
            linkMasses=[0.0] * len(link_collision_shapes),
            linkCollisionShapeIndices=link_collision_shapes,
            linkVisualShapeIndices=link_visual_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=[[0.0, 0.0, 0.0]] * len(link_collision_shapes),
            linkInertialFrameOrientations=[identity_quat] * len(link_collision_shapes),
            linkParentIndices=[0] * len(link_collision_shapes),
            linkJointTypes=[bullet.JOINT_FIXED] * len(link_collision_shapes),
            linkJointAxis=[[0.0, 0.0, 1.0]] * len(link_collision_shapes),
            physicsClientId=self.client_id,
        )

        chassis_contact_friction = float(friction)
        paddle_contact_friction = max(0.32, float(friction) * 1.4)
        bullet.changeDynamics(
            body_id,
            -1,
            lateralFriction=chassis_contact_friction,
            rollingFriction=ROBOT_CONTACT_ROLLING_FRICTION,
            spinningFriction=ROBOT_CONTACT_SPINNING_FRICTION,
            restitution=float(restitution),
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )
        bullet.setCollisionFilterGroupMask(
            body_id,
            -1,
            collisionFilterGroup=ROBOT_CHASSIS_COLLISION_GROUP,
            collisionFilterMask=ROBOT_CHASSIS_MASK,
            physicsClientId=self.client_id,
        )
        bullet.changeDynamics(
            body_id,
            0,
            lateralFriction=ROBOT_BUMPER_FRICTION,
            rollingFriction=0.0,
            spinningFriction=0.0,
            restitution=ROBOT_BUMPER_RESTITUTION,
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )
        bullet.setCollisionFilterGroupMask(
            body_id,
            0,
            collisionFilterGroup=ROBOT_BUMPER_COLLISION_GROUP,
            collisionFilterMask=ROBOT_BUMPER_MASK,
            physicsClientId=self.client_id,
        )
        for link_index in (1, 2, 3):
            bullet.changeDynamics(
                body_id,
                link_index,
                lateralFriction=paddle_contact_friction,
                rollingFriction=0.0,
                spinningFriction=0.0,
                restitution=float(restitution),
                linearDamping=0.0,
                angularDamping=0.0,
                physicsClientId=self.client_id,
            )
            bullet.setCollisionFilterGroupMask(
                body_id,
                link_index,
                collisionFilterGroup=0,
                collisionFilterMask=0,
                physicsClientId=self.client_id,
            )

        half_w = size_v.x * 0.5
        half_h = size_v.y * 0.5
        verts = [
            Vec2(-half_w, -half_h),
            Vec2(half_w, -half_h),
            Vec2(half_w, half_h),
            Vec2(-half_w, half_h),
        ]
        moment_z = math.inf
        if mass_kg > 0.0:
            moment_z_si = (mass_kg * ((width_m * width_m) + (depth_m * depth_m))) / 12.0
            moment_z = _inertia_si_to_sim(moment_z_si)

        body = PhysicsBody(
            self,
            body_id,
            mass=mass_kg,
            moment=moment_z,
            z_m=z_center_m,
            dynamic=True,
            planar_lock=False,
            gravity_scale=1.0,
        )
        shape = PhysicsShape(
            body=body,
            kind="poly",
            local_vertices=verts,
            friction=chassis_contact_friction,
            elasticity=float(restitution),
        )
        shape.render_wheel_offsets_px = []
        shape.render_wheel_radius_px = meters_to_px(wheel_radius_m)
        shape.render_chassis_size_px = (
            size_v.x,
            size_v.y,
        )
        shape.robot_wheel_link_indices = ()
        shape.robot_bumper_link_indices = (0,)
        shape.loader_paddle_link_indices = (1, 2, 3)
        shape.loader_paddle_collision_active = False
        shape.render_kind = "robot"
        shape.render_size_px = (size_v.x, size_v.y)
        shape.render_height_m = ROBOT_BODY_HEIGHT_M
        shape.render_rgba = ROBOT_CHASSIS_VISUAL_RGBA
        self._register_body_and_shape(body, shape)
        return shape

    def create_static_wedge_bar(
        self,
        center_px,
        size_px,
        *,
        angle: float = 0.0,
        friction: float = 1.0,
        restitution: float = 0.0,
        height_m: float = PARK_ZONE_WALL_HEIGHT_M,
    ) -> PhysicsShape:
        center_v = as_vec2(center_px)
        size_v = as_vec2(size_px)
        collision_shape = self._cached_mesh_shape(
            PARK_ZONE_BAR_WEDGE_ASSET_PATH,
            (px_to_meters(size_v.x), px_to_meters(size_v.y), height_m),
        )
        visual_shape = self._cached_mesh_visual(
            PARK_ZONE_BAR_WEDGE_ASSET_PATH,
            (px_to_meters(size_v.x), px_to_meters(size_v.y), height_m),
        )
        body_id = bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=collision_shape,
            baseVisualShapeIndex=visual_shape,
            basePosition=[
                px_to_meters(center_v.x),
                px_to_meters(center_v.y),
                0.0,
            ],
            baseOrientation=_yaw_to_quat(angle),
            physicsClientId=self.client_id,
        )
        bullet.changeDynamics(
            body_id,
            -1,
            lateralFriction=float(friction),
            restitution=float(restitution),
            rollingFriction=0.0,
            spinningFriction=0.0,
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )
        bullet.setCollisionFilterGroupMask(
            body_id,
            -1,
            collisionFilterGroup=PARK_ZONE_COLLISION_GROUP,
            collisionFilterMask=0xFFFF,
            physicsClientId=self.client_id,
        )

        half_len = size_v.x * 0.5
        half_width = size_v.y * 0.5
        verts = [
            Vec2(-half_len, -half_width),
            Vec2(half_len, -half_width),
            Vec2(half_len, half_width),
            Vec2(-half_len, half_width),
        ]
        body = PhysicsBody(
            self,
            body_id,
            mass=0.0,
            moment=math.inf,
            z_m=0.0,
            dynamic=False,
            planar_lock=True,
            gravity_scale=0.0,
        )
        shape = PhysicsShape(
            body=body,
            kind="poly",
            local_vertices=verts,
            friction=float(friction),
            elasticity=float(restitution),
        )
        shape.render_kind = "wedge_bar"
        shape.render_size_px = (size_v.x, size_v.y)
        shape.render_height_m = float(height_m)
        shape.render_rgba = DEFAULT_WEDGE_VISUAL_RGBA
        self._register_body_and_shape(body, shape)
        return shape

    def create_block(
        self,
        pos_px,
        *,
        color: str = "red",
        air: bool = False,
        elasticity: float = BLOCK_CONTACT_RESTITUTION,
        friction: Optional[float] = None,
        z_center_m: Optional[float] = None,
    ) -> PhysicsShape:
        center_v = as_vec2(pos_px)
        if z_center_m is None:
            z_center_m = BLOCK_BODY_HEIGHT_M * 0.5
        body_id = bullet.createMultiBody(
            baseMass=BLOCK_MASS,
            baseCollisionShapeIndex=self._triball_collision_shape(),
            baseVisualShapeIndex=self._triball_visual_shape(color),
            basePosition=[
                px_to_meters(center_v.x),
                px_to_meters(center_v.y),
                z_center_m,
            ],
            baseOrientation=(0.0, 0.0, 0.0, 1.0),
            physicsClientId=self.client_id,
        )
        contact_friction = BLOCK_CONTACT_FRICTION if friction is None else friction
        bullet.changeDynamics(
            body_id,
            -1,
            lateralFriction=float(contact_friction),
            rollingFriction=BLOCK_CONTACT_ROLLING_FRICTION,
            spinningFriction=BLOCK_CONTACT_SPINNING_FRICTION,
            restitution=float(elasticity),
            linearDamping=0.0,
            angularDamping=0.0,
            physicsClientId=self.client_id,
        )
        bullet.setCollisionFilterGroupMask(
            body_id,
            -1,
            collisionFilterGroup=BLOCK_COLLISION_GROUP,
            collisionFilterMask=BLOCK_COLLISION_MASK,
            physicsClientId=self.client_id,
        )

        approx_radius_m = 0.25 * (TRIBALL_MESH.width_m + TRIBALL_MESH.depth_m)
        try:
            bullet.changeDynamics(
                body_id,
                -1,
                ccdSweptSphereRadius=approx_radius_m * 0.65,
                contactProcessingThreshold=0.0,
                physicsClientId=self.client_id,
            )
        except TypeError:
            pass

        moment_z = _inertia_si_to_sim(0.4 * BLOCK_MASS * (approx_radius_m * approx_radius_m))
        body = PhysicsBody(
            self,
            body_id,
            mass=BLOCK_MASS,
            moment=moment_z,
            z_m=z_center_m,
            dynamic=True,
            planar_lock=False,
            gravity_scale=1.0,
        )
        shape = PhysicsShape(
            body=body,
            kind="poly",
            local_vertices=list(TRIBALL_VERTICES),
            friction=float(contact_friction),
            elasticity=float(elasticity),
        )
        shape.is_block = True
        shape.block_color = color
        shape.block_pickup_radius_px = BLOCK_RADIUS_PX
        shape.air_timer = AIR_TIME_SEC if air else 0.0
        shape.mesh_source = TRIBALL_MESH.source_path
        shape.render_sample_vertices_px3d = TRIBALL_MESH.render_sample_vertices_px3d
        shape.render_kind = "triball"
        shape.render_height_m = BLOCK_BODY_HEIGHT_M
        shape.render_rgba = TRIBALL_BLUE_VISUAL_RGBA if color == "blue" else TRIBALL_RED_VISUAL_RGBA
        shape.rest_timer = 0.0

        self._register_body_and_shape(body, shape)
        self.block_shapes.append(shape)
        return shape

    def remove(self, *items):
        body_ids = set()
        for item in items:
            if isinstance(item, PhysicsShape):
                body_ids.add(item.body.body_id)
            elif isinstance(item, PhysicsBody):
                body_ids.add(item.body_id)

        for body_id in body_ids:
            body = self._uid_to_body.pop(body_id, None)
            shape = self._uid_to_shape.pop(body_id, None)
            if body is None or shape is None:
                continue

            if body in self.bodies:
                self.bodies.remove(body)
            if body in self._dynamic_bodies:
                self._dynamic_bodies.remove(body)
            if shape in self.shapes:
                self.shapes.remove(shape)
            if shape in self.block_shapes:
                self.block_shapes.remove(shape)

            bullet.removeBody(body_id, physicsClientId=self.client_id)

    def _target_solver_iterations(self) -> int:
        block_count = len(self.block_shapes)
        if block_count >= PYBULLET_SOLVER_HIGH_BLOCK_COUNT:
            return PYBULLET_SOLVER_ITERS_HIGH_LOAD
        if block_count >= PYBULLET_SOLVER_MEDIUM_BLOCK_COUNT:
            return PYBULLET_SOLVER_ITERS_MEDIUM_LOAD
        return PYBULLET_SOLVER_ITERS

    def _stabilize_block_shapes(self):
        min_xy = BLOCK_CONTAINMENT_MARGIN_PX
        max_xy = FIELD_SIZE_PX - BLOCK_CONTAINMENT_MARGIN_PX

        for shape in list(self.block_shapes):
            body = shape.body
            pos = body.position
            new_x = min(max(pos.x, min_xy), max_xy)
            new_y = min(max(pos.y, min_xy), max_xy)
            new_z = body.z
            needs_position_reset = new_x != pos.x or new_y != pos.y

            if body.z < BLOCK_MIN_CENTER_Z_M:
                new_z = BLOCK_RESCUE_CENTER_Z_M
                needs_position_reset = True
            elif body.z > BLOCK_MAX_CENTER_Z_M:
                new_z = BLOCK_RESCUE_CENTER_Z_M
                needs_position_reset = True

            vel = body.velocity
            lin_x = vel.x / PX_PER_M
            lin_y = vel.y / PX_PER_M
            lin_z = body._linear_velocity_z
            needs_velocity_reset = False

            speed = vel.length
            if speed > BLOCK_MAX_SPEED_PXPS:
                scale = BLOCK_MAX_SPEED_PXPS / max(speed, 1e-6)
                lin_x *= scale
                lin_y *= scale
                needs_velocity_reset = True

            if abs(lin_z) > BLOCK_MAX_VERTICAL_SPEED_MPS:
                lin_z = BLOCK_MAX_VERTICAL_SPEED_MPS if lin_z > 0.0 else -BLOCK_MAX_VERTICAL_SPEED_MPS
                needs_velocity_reset = True

            if needs_position_reset and lin_z < 0.0:
                lin_z = 0.0
                needs_velocity_reset = True

            if needs_position_reset:
                bullet.resetBasePositionAndOrientation(
                    body.body_id,
                    [px_to_meters(new_x), px_to_meters(new_y), new_z],
                    body.quat,
                    physicsClientId=self.client_id,
                )

            if needs_position_reset or needs_velocity_reset:
                bullet.resetBaseVelocity(
                    body.body_id,
                    linearVelocity=[lin_x, lin_y, lin_z],
                    angularVelocity=body.angular_velocity_world,
                    physicsClientId=self.client_id,
                )
                body.sync_from_bullet()

    def step(self, dt: float):
        if not self.connected:
            return

        target_solver_iterations = self._target_solver_iterations()
        if abs(dt - self.fixed_dt) > 1e-9 or target_solver_iterations != self._solver_iterations:
            self.fixed_dt = float(dt)
            self._solver_iterations = target_solver_iterations
            bullet.setPhysicsEngineParameter(
                fixedTimeStep=self.fixed_dt,
                numSolverIterations=self._solver_iterations,
                numSubSteps=0,
                useSplitImpulse=1,
                splitImpulsePenetrationThreshold=-0.01,
                contactBreakingThreshold=PYBULLET_CONTACT_BREAKING_THRESHOLD_M,
                restitutionVelocityThreshold=PYBULLET_RESTITUTION_VELOCITY_THRESHOLD,
                physicsClientId=self.client_id,
            )

        for body in self._dynamic_bodies:
            world_pos = [
                px_to_meters(body.position.x),
                px_to_meters(body.position.y),
                body.z,
            ]

            if body.gravity_scale != 1.0:
                bullet.applyExternalForce(
                    body.body_id,
                    -1,
                    [0.0, 0.0, -body.mass * STD_GRAVITY * (body.gravity_scale - 1.0)],
                    world_pos,
                    bullet.WORLD_FRAME,
                    physicsClientId=self.client_id,
                )

            if body.force.length > 1e-9:
                bullet.applyExternalForce(
                    body.body_id,
                    -1,
                    [body.force.x / PX_PER_M, body.force.y / PX_PER_M, 0.0],
                    world_pos,
                    bullet.WORLD_FRAME,
                    physicsClientId=self.client_id,
                )
            if abs(body.torque) > 1e-9:
                bullet.applyExternalTorque(
                    body.body_id,
                    -1,
                    [0.0, 0.0, body.torque / (PX_PER_M * PX_PER_M)],
                    bullet.WORLD_FRAME,
                    physicsClientId=self.client_id,
                )

        bullet.stepSimulation(physicsClientId=self.client_id)

        for body in self._dynamic_bodies:
            body.sync_from_bullet()

        self._stabilize_block_shapes()

        for body in self._dynamic_bodies:
            body.clear_external_loads()


def create_space():
    return PhysicsWorld()


def _create_parking_zone(space: PhysicsWorld, *, center_x: float, attach_to: str, color: str):
    outer = PARK_ZONE_OUTER_SIZE_PX
    wall_t = PARK_ZONE_WALL_THICKNESS_PX
    half_outer = outer * 0.5
    bar_size = (outer, wall_t)

    if attach_to == "top":
        side_center_y = half_outer
        cross_center_y = outer - (wall_t * 0.5)
    else:
        side_center_y = FIELD_SIZE_PX - half_outer
        cross_center_y = FIELD_SIZE_PX - outer + (wall_t * 0.5)

    side_dx = half_outer - (wall_t * 0.5)
    side_specs = (
        ((center_x - side_dx, side_center_y), math.pi * 0.5, Vec2(-1.0, 0.0)),
        ((center_x + side_dx, side_center_y), -math.pi * 0.5, Vec2(1.0, 0.0)),
    )
    for center, angle, outward in side_specs:
        shape = space.create_static_wedge_bar(
            center,
            bar_size,
            angle=angle,
            friction=PARK_ZONE_CONTACT_FRICTION,
            restitution=PARK_ZONE_CONTACT_RESTITUTION,
            height_m=PARK_ZONE_COLLISION_HEIGHT_M,
        )
        shape.is_parking_zone_edge = True
        shape.parking_zone_color = color
        shape.field_render_role = "parking_zone_edge"
        shape.parking_zone_edge_role = "side"
        shape.parking_zone_outward = outward
        space.field_feature_shapes.append(shape)

    outward = Vec2(0.0, 1.0) if attach_to == "top" else Vec2(0.0, -1.0)
    cross_shape = space.create_static_wedge_bar(
        (center_x, cross_center_y),
        bar_size,
        angle=0.0 if attach_to == "top" else math.pi,
        friction=PARK_ZONE_CONTACT_FRICTION,
        restitution=PARK_ZONE_CONTACT_RESTITUTION,
        height_m=PARK_ZONE_COLLISION_HEIGHT_M,
    )
    cross_shape.is_parking_zone_edge = True
    cross_shape.parking_zone_color = color
    cross_shape.field_render_role = "parking_zone_edge"
    cross_shape.parking_zone_edge_role = "cross"
    cross_shape.parking_zone_outward = outward
    space.field_feature_shapes.append(cross_shape)


def add_field_boundaries(space: PhysicsWorld):
    thickness = FIELD_WALL_THICKNESS_PX
    half = thickness * 0.5

    floor_size_px = FIELD_SIZE_PX + (2.0 * thickness)
    floor = space.create_static_box(
        (FIELD_SIZE_PX * 0.5, FIELD_SIZE_PX * 0.5),
        (floor_size_px, floor_size_px),
        friction=FIELD_FLOOR_FRICTION,
        restitution=FIELD_FLOOR_RESTITUTION,
        height_m=FLOOR_THICKNESS_M,
        z_center_m=-FLOOR_THICKNESS_M * 0.5,
    )
    floor.is_field_floor = True

    top_wall = space.create_static_box(
        (FIELD_SIZE_PX * 0.5, -half),
        (FIELD_SIZE_PX + (2.0 * thickness), thickness),
        friction=1.05,
        restitution=FIELD_WALL_RESTITUTION,
        height_m=FIELD_WALL_HEIGHT_M,
        z_center_m=FIELD_WALL_HEIGHT_M * 0.5,
    )
    top_wall.is_field_wall = True

    bottom_wall = space.create_static_box(
        (FIELD_SIZE_PX * 0.5, FIELD_SIZE_PX + half),
        (FIELD_SIZE_PX + (2.0 * thickness), thickness),
        friction=1.05,
        restitution=FIELD_WALL_RESTITUTION,
        height_m=FIELD_WALL_HEIGHT_M,
        z_center_m=FIELD_WALL_HEIGHT_M * 0.5,
    )
    bottom_wall.is_field_wall = True

    left_wall = space.create_static_box(
        (-half, FIELD_SIZE_PX * 0.5),
        (thickness, FIELD_SIZE_PX + (2.0 * thickness)),
        friction=1.05,
        restitution=FIELD_WALL_RESTITUTION,
        height_m=FIELD_WALL_HEIGHT_M,
        z_center_m=FIELD_WALL_HEIGHT_M * 0.5,
    )
    left_wall.is_field_wall = True

    right_wall = space.create_static_box(
        (FIELD_SIZE_PX + half, FIELD_SIZE_PX * 0.5),
        (thickness, FIELD_SIZE_PX + (2.0 * thickness)),
        friction=1.05,
        restitution=FIELD_WALL_RESTITUTION,
        height_m=FIELD_WALL_HEIGHT_M,
        z_center_m=FIELD_WALL_HEIGHT_M * 0.5,
    )
    right_wall.is_field_wall = True

    _create_parking_zone(space, center_x=FIELD_SIZE_PX * 0.5, attach_to="top", color="blue")
    _create_parking_zone(space, center_x=FIELD_SIZE_PX * 0.5, attach_to="bottom", color="red")


def spawn_block(
    space: PhysicsWorld,
    pos_px,
    color="red",
    air=False,
    elasticity=0.10,
    friction=None,
    z_center_m: Optional[float] = None,
):
    return space.create_block(
        pos_px,
        color=color,
        air=air,
        elasticity=elasticity,
        friction=friction,
        z_center_m=z_center_m,
    )


def spawn_initial_blocks(space: PhysicsWorld):
    for (x_norm, y_norm, color) in INITIAL_BLOCKS_LAYOUT:
        x = x_norm * FIELD_SIZE_PX
        y = y_norm * FIELD_SIZE_PX
        spawn_block(space, (x, y), color=color, air=False)


def apply_ground_friction_and_wobble(space: PhysicsWorld, dt: float):
    for shape in space.block_shapes:
        air_t = getattr(shape, "air_timer", 0.0)
        if air_t > 0.0:
            shape.air_timer = max(0.0, air_t - dt)

        body = shape.body
        speed = body.velocity.length
        ang_world = body.angular_velocity_world
        ang_mag = math.sqrt(
            (ang_world[0] * ang_world[0])
            + (ang_world[1] * ang_world[1])
            + (ang_world[2] * ang_world[2])
        )

        resting_height = BLOCK_BODY_HEIGHT_M * 0.5
        on_ground = body.z <= (resting_height + 0.002)

        if on_ground and air_t <= 0.0 and speed > BLOCK_ROLL_ASSIST_MIN_SPEED_PX:
            blend = 1.0 - math.exp(-BLOCK_ROLL_ASSIST_GAIN * dt)
            vx_mps = body.velocity.x / PX_PER_M
            vy_mps = body.velocity.y / PX_PER_M
            desired_omega_x = -vy_mps / max(1e-6, BLOCK_EFFECTIVE_ROLL_RADIUS_M)
            desired_omega_y = vx_mps / max(1e-6, BLOCK_EFFECTIVE_ROLL_RADIUS_M)
            omega_err_x = desired_omega_x - ang_world[0]
            omega_err_y = desired_omega_y - ang_world[1]
            omega_err_mag = math.sqrt((omega_err_x * omega_err_x) + (omega_err_y * omega_err_y))
            if omega_err_mag > BLOCK_ROLL_ASSIST_ANG_ERROR_MIN:
                new_ang = [
                    ang_world[0] + (omega_err_x * blend),
                    ang_world[1] + (omega_err_y * blend),
                    ang_world[2],
                ]
                bullet.resetBaseVelocity(
                    body.body_id,
                    linearVelocity=[
                        body.velocity.x / PX_PER_M,
                        body.velocity.y / PX_PER_M,
                        body._linear_velocity_z,
                    ],
                    angularVelocity=new_ang,
                    physicsClientId=space.client_id,
                )
                ang_world = tuple(new_ang)
                ang_mag = math.sqrt(
                    (ang_world[0] * ang_world[0])
                    + (ang_world[1] * ang_world[1])
                    + (ang_world[2] * ang_world[2])
                )

        if (
            on_ground
            and speed < BLOCK_SLEEP_SPEED_PX
            and abs(body.angular_velocity) < BLOCK_SLEEP_YAW_RATE
            and ang_mag < BLOCK_SLEEP_ANG_WORLD
            and abs(body._linear_velocity_z) < BLOCK_SLEEP_LINEAR_Z
        ):
            shape.rest_timer = getattr(shape, "rest_timer", 0.0) + dt
            if shape.rest_timer >= BLOCK_SLEEP_TIME:
                bullet.resetBaseVelocity(
                    body.body_id,
                    linearVelocity=[0.0, 0.0, 0.0],
                    angularVelocity=[0.0, 0.0, 0.0],
                    physicsClientId=space.client_id,
                )
        else:
            shape.rest_timer = 0.0
