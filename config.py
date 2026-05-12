import math

# ============================
# SIMULATION CONSTANTS
# ============================

PPM = 5.0
FIELD_SIZE_IN = 144.0
FIELD_SIZE_PX = int(FIELD_SIZE_IN * PPM)

SCREEN_SIZE = (FIELD_SIZE_PX + 260, FIELD_SIZE_PX)

FPS = 60
PHYSICS_HZ = 60
FIXED_DT = 1.0 / PHYSICS_HZ
MAX_ACCUM = 0.25

MATCH_TIME_SEC = 60.0

# ============================
# INPUT / CONTROLLERS
# ============================

GAMEPAD_DEADZONE = 0.01
GAMEPAD_AXIS_EXPONENT = 1.45

# VEX V5 via USB on PC:
# VEX documentation labels the sticks as Axis1/2 on the right joystick
# and Axis3/4 on the left joystick. The default guess below assumes
# pygame exposes them more like a standard HID gamepad:
# left stick = axes 0/1, right stick = axes 2/3.
VEX_CONTROLLER_NAME_HINTS = ("vex", "v5")
VEX_FORWARD_AXIS = 1
VEX_TURN_AXIS = 2
VEX_INVERT_FORWARD = True
VEX_INVERT_TURN = False

# Button ids can vary slightly by HID mapping, so these stay easy to tweak.
# Assumed default mapping:
# X can appear as 2 or 3 depending on how Windows/SDL exposes the VEX HID.
# R1 = 5, R2 = 7, down arrow = hat 0 / y=-1.
VEX_BUTTON_INTAKE_TOGGLE = (2, 3)
VEX_BUTTON_DUMP_HOLD = (5, 7)
VEX_LOADER_HAT_INDEX = 0
VEX_LOADER_HAT_VECTOR = (0, -1)

# Generic fallback if the controller is exposed with a non-VEX name.
GENERIC_FORWARD_AXIS = 1
GENERIC_TURN_AXIS = 0
GENERIC_INVERT_FORWARD = True
GENERIC_INVERT_TURN = False
GENERIC_BUTTON_INTAKE_TOGGLE = (2, 3)
GENERIC_BUTTON_DUMP_HOLD = (5, 7)
GENERIC_LOADER_HAT_INDEX = 0
GENERIC_LOADER_HAT_VECTOR = (0, -1)

# ============================
# UNIT CONVERSION
# ============================

IN_TO_M = 0.0254
M_TO_IN = 1.0 / IN_TO_M
M_PER_PX = IN_TO_M / PPM
PX_PER_M = 1.0 / M_PER_PX
STD_GRAVITY = 9.80665


def clamp(x, a, b):
    return a if x < a else b if x > b else x


def lerp(a, b, t):
    return a + (b - a) * t


def rpm01(rpm, rmin, rmax):
    rpm = clamp(rpm, rmin, rmax)
    return (rpm - rmin) / float(rmax - rmin)


def inches_to_px(inches: float) -> float:
    return inches * PPM


def inches_to_m(inches: float) -> float:
    return inches * IN_TO_M


def meters_to_px(meters: float) -> float:
    return meters * PX_PER_M


def px_to_meters(px: float) -> float:
    return px * M_PER_PX


def mps_to_pxps(mps: float) -> float:
    return meters_to_px(mps)


def pxps_to_mps(pxps: float) -> float:
    return px_to_meters(pxps)


def mps2_to_pxps2(mps2: float) -> float:
    return meters_to_px(mps2)


def newton_to_sim_force(newton: float) -> float:
    return newton * PX_PER_M


def newton_meter_to_sim_torque(newton_meter: float) -> float:
    return newton_meter * (PX_PER_M ** 2)


# ============================
# PRIMARY RPM TUNING
# ============================

INTAKE_RPM = 900

# Kept as the default drivetrain cartridge/output speed.
DRIVETRAIN_DEFAULT_RPM = 450.0

# Backward-compatible alias used by older code paths.
DRIVETRAIN_RPM = DRIVETRAIN_DEFAULT_RPM


# ============================
# TRIBALLS / GAME PIECES
# ============================

# Approximation of the VEX triball:
# - 82 mm across opposite flats
# - 98 mm across opposite corners
# - 18-sided rounded cross-section
# In 2D we use an effective support diameter plus a rounded 18-gon.
BLOCK_FLAT_DIAM_MM = 82.0
BLOCK_CORNER_DIAM_MM = 98.0
BLOCK_EFFECTIVE_DIAM_MM = 0.5 * (BLOCK_FLAT_DIAM_MM + BLOCK_CORNER_DIAM_MM)

BLOCK_DIAM_IN = (BLOCK_EFFECTIVE_DIAM_MM / 25.4)
BLOCK_RADIUS_PX = 0.5 * inches_to_px(BLOCK_DIAM_IN)
BLOCK_MASS = 0.04

BLOCK_CROSS_SECTION_SIDES = 18
BLOCK_CORNER_RADIUS_MM = 49.0
BLOCK_FLAT_RADIUS_MM = 41.0
BLOCK_EDGE_ROUND_MM = 3.0
BLOCK_EDGE_ROUND_PX = (BLOCK_EDGE_ROUND_MM / 25.4) * PPM

# Plastic triball on VEX foam tiles: moderate surface grip,
# noticeable rolling resistance, low bounce.
BLOCK_SURFACE_FRICTION = 0.68
BLOCK_ROLLING_RESISTANCE_COEFF = 0.040
BLOCK_LINEAR_DAMP_GROUND = 1.75
BLOCK_LINEAR_DAMP_AIR = 0.45
BLOCK_ANGULAR_DAMP_GROUND = 0.4
BLOCK_ANGULAR_DAMP_AIR = 1.8
BLOCK_STOP_SPEED_PX = 2.5
BLOCK_STOP_ANG_VEL = 0.22

# Mild deterministic faceting ripple to hint the non-round rolling shape
# without the old large random wobble.
BLOCK_FACET_RIPPLE_ACCEL = 62.0
BLOCK_FACET_RIPPLE_SPIN = 2.6
BLOCK_FACET_YAW_RATE = 2.8
BLOCK_WANDER_ACCEL = 54.0
BLOCK_WANDER_SPIN = 4.4
BLOCK_WANDER_YAW_RATE = 5.2
BLOCK_WANDER_HOLD_MIN = 0.05
BLOCK_WANDER_HOLD_MAX = 0.16


# ============================
# ROBOT GEOMETRY / DEFAULTS
# ============================

ROBOT_SIZE_IN = 12.0
ROBOT_SIZE_PX = inches_to_px(ROBOT_SIZE_IN)

# Requested default robot mass.
ROBOT_DEFAULT_MASS_KG = 9.0

# Backward-compatible alias.
ROBOT_MASS = ROBOT_DEFAULT_MASS_KG


# ============================
# DRIVETRAIN DEFAULT PHYSICS
# ============================

DRIVETRAIN_MOTOR_POWER_W = 11.0
DRIVETRAIN_MOTOR_COUNT = 6
DRIVE_WHEEL_DEFAULT_DIAMETER_IN = 4.0

# Distance between left and right wheel centerlines.
DRIVE_TRACK_WIDTH_IN = 10.5

# Motor / gearbox / wheel efficiency lumped together.
DRIVE_MOTOR_EFFICIENCY = 0.82

# Command shaping and damping.
DRIVE_INPUT_TURN_GAIN = 0.86
DRIVE_SPEED_CONTROL_TIME_CONSTANT = 0.12
DRIVE_ACTIVE_BRAKE_FACTOR = 0.90
DRIVE_LINEAR_DRAG_COEFF = 0.30
DRIVE_ANGULAR_DRAG_COEFF = 1.10

# Contact model for omniwheels:
# high longitudinal grip, much lower lateral grip.
DRIVE_LONGITUDINAL_MU = 1.08
DRIVE_LATERAL_MU = 0.07
DRIVE_LATERAL_TIME_CONSTANT_SEC = 0.32
DRIVE_ROLLING_RESISTANCE_COEFF = 0.095
DRIVE_YAW_SCRUB_COEFF = 0.06
DRIVE_YAW_VISCOUS_DAMPING = 1.35
DRIVE_MOVING_TURN_REDUCTION = 0.20
DRIVE_MOVING_LATERAL_HOLD_REDUCTION = 0.88


def drivetrain_free_speed_mps(rpm: float, wheel_diameter_in: float) -> float:
    wheel_radius_m = inches_to_m(wheel_diameter_in) * 0.5
    return (rpm * math.tau / 60.0) * wheel_radius_m


# Useful default reference value for HUD/debugging.
ROBOT_FREE_SPEED_PX = mps_to_pxps(
    drivetrain_free_speed_mps(DRIVETRAIN_DEFAULT_RPM, DRIVE_WHEEL_DEFAULT_DIAMETER_IN)
)


# ============================
# INTAKE / DUMP (FROM INTAKE_RPM)
# ============================

INTAKE_CAPACITY = 8

_it = rpm01(INTAKE_RPM, 300, 1500)

INTAKE_TIME_SEC = 0.5 * lerp(0.85, 0.18, _it)
INTAKE_JITTER = 0.12 * INTAKE_TIME_SEC

DUMP_INTERVAL_SEC = lerp(0.34, 0.09, _it)
DUMP_FLOOR_SPEED = lerp(35.0, 120.0, _it)


# ============================
# GOAL / CAPACITY
# ============================

LONG_GOAL_CAPACITY = 15
MIDDLE_GOAL_CAPACITY = 7


# ============================
# PARK ZONES
# ============================

# Official V5RC Push Back park zone drawing:
# 18.86" outer, 14.86" clear opening, 2.0" rail width, 1.0" height.
PARK_ZONE_OUTER_SIZE_IN = 18.86
PARK_ZONE_INNER_SIZE_IN = 14.86
PARK_ZONE_WALL_THICKNESS_IN = 2.0
PARK_ZONE_WALL_HEIGHT_IN = 1.0


# ============================
# BLOCK FRICTION / AIR
# ============================

GROUND_FRICTION_MU = 6.0
AIR_FRICTION_MU = 1.0
AIR_TIME_SEC = 0.4


# ============================
# TUBES: PISTON + FRICTION
# ============================

TUBE_FRICTION_RATE = 6.5
TUBE_BALL_CONTACT_FRICTION = 22.0
TUBE_BALL_CONTACT_DAMP = 11.0

TUBE_ENTRY_SPEED = lerp(0.15, 0.50, _it)
TUBE_ENTRY_SPEED_JITTER = 0.15
TUBE_ENTRY_POS_JITTER = 0.35

TUBE_PUSH_ACCEL = lerp(14.0, 40.0, _it)
TUBE_PUSH_TIME = 0.25

TUBE_RESTITUTION = 0.18
TUBE_SOLVER_ITERS = 8
TUBE_WALL_DAMP = 0.15
TUBE_EXIT_MIN_SPEED_PX = 42.0
TUBE_EXIT_SPEED_GAIN = 0.92
TUBE_EXIT_SIDE_JITTER_GAIN = 0.70

TUBE_FILL_FRICTION_GAIN = 2.2
TUBE_FILL_FRICTION_POWER = 1.8


# ============================
# GOAL EXITS
# ============================

EXIT_OFFSET_JITTER_PX = 1.3 * BLOCK_RADIUS_PX
EXIT_VEL_MAIN_JITTER = 0.35
EXIT_VEL_SIDE_JITTER = 90.0
EXIT_SPIN_JITTER = 14.0
EXIT_ELASTICITY = 0.45


# ============================
# GROUND / AIR WOBBLE
# ============================

GROUND_WOBBLE_ACCEL = 340.0
GROUND_WOBBLE_SPIN_ACCEL = 26.0
GROUND_WOBBLE_MIN_SPEED = 4.0
GROUND_WOBBLE_MAX_SPEED = 320.0
GROUND_WOBBLE_HOLD_MIN = 0.15
GROUND_WOBBLE_HOLD_MAX = 0.45

AIR_WOBBLE_ACCEL = 110.0
AIR_WOBBLE_SPIN_ACCEL = 18.0
AIR_WOBBLE_HOLD_MIN = 0.10
AIR_WOBBLE_HOLD_MAX = 0.30


# ============================
# LOADER
# ============================

LOADER_DIAM_IN = 5.0
LOADER_RADIUS_PX = (LOADER_DIAM_IN * PPM) * 0.5
LOADER_CAPACITY = 6

LOADER_PADDLE_LEN_IN = 7.0
LOADER_PADDLE_W_IN = 10.0
LOADER_PADDLE_T_IN = 1.2

LOADER_PADDLE_LEN_PX = LOADER_PADDLE_LEN_IN * PPM
LOADER_PADDLE_W_PX = LOADER_PADDLE_W_IN * PPM
LOADER_PADDLE_T_PX = LOADER_PADDLE_T_IN * PPM


# ============================
# INITIAL BLOCK LAYOUT
# ============================

def _layout_point(x_in: float, y_in: float, color: str):
    return (x_in / FIELD_SIZE_IN, y_in / FIELD_SIZE_IN, color)


def _append_cluster_2x2(layout, cx_in: float, cy_in: float, spacing_in: float, top_left: str, top_right: str, bot_left: str, bot_right: str):
    half = spacing_in * 0.5
    layout.extend(
        [
            _layout_point(cx_in - half, cy_in - half, top_left),
            _layout_point(cx_in + half, cy_in - half, top_right),
            _layout_point(cx_in - half, cy_in + half, bot_left),
            _layout_point(cx_in + half, cy_in + half, bot_right),
        ]
    )


def _append_grid(layout, xs_in, ys_in, color: str):
    for y_in in ys_in:
        for x_in in xs_in:
            layout.append(_layout_point(x_in, y_in, color))


_skills_layout = []

# Left / right long-goal starter pairs.
_skills_layout.extend(
    [
        _layout_point(24.0, 70.25, "red"),
        _layout_point(24.0, 73.75, "blue"),
        _layout_point(120.0, 70.25, "red"),
        _layout_point(120.0, 73.75, "blue"),
    ]
)

# Four mixed 2x2 stacks around the center goals.
for cluster_center in (
    (50.0, 50.0),
    (91.0, 50.0),
    (50.0, 91.0),
    (91.0, 91.0),
):
    _append_cluster_2x2(_skills_layout, cluster_center[0], cluster_center[1], 3.85, "blue", "red", "red", "blue")

# Blue park zone at the top side of the rotated field.
_append_grid(_skills_layout, (68.4, 72.0, 75.6), (11.5, 14.7), "red")

# Red park zone at the bottom side of the rotated field.
_append_grid(_skills_layout, (68.4, 72.0, 75.6), (128.8, 132.3), "blue")

INITIAL_BLOCKS_LAYOUT = _skills_layout
