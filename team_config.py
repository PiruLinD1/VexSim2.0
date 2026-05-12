import json
from dataclasses import asdict, dataclass
from pathlib import Path

from config import (
    DRIVETRAIN_DEFAULT_RPM,
    DRIVE_WHEEL_DEFAULT_DIAMETER_IN,
    GAMEPAD_AXIS_EXPONENT,
    GAMEPAD_DEADZONE,
    GENERIC_FORWARD_AXIS,
    GENERIC_INVERT_FORWARD,
    GENERIC_INVERT_TURN,
    GENERIC_TURN_AXIS,
    INTAKE_RPM,
    ROBOT_DEFAULT_MASS_KG,
    ROBOT_SIZE_IN,
    VEX_BUTTON_DUMP_HOLD,
    VEX_BUTTON_INTAKE_TOGGLE,
    VEX_FORWARD_AXIS,
    VEX_INVERT_FORWARD,
    VEX_INVERT_TURN,
    VEX_LOADER_HAT_INDEX,
    VEX_LOADER_HAT_VECTOR,
    VEX_TURN_AXIS,
    clamp,
)


TEAM_CONFIG_PATH = Path(__file__).resolve().with_name("team_configs.json")

CM_PER_IN = 2.54
LB_PER_KG = 2.2046226218

LENGTH_UNITS = ("in", "cm")
MASS_UNITS = ("kg", "lb")

MIN_ROBOT_SIZE_IN = 6.0
MAX_ROBOT_SIZE_IN = 24.0
MIN_MASS_KG = 1.0
MAX_MASS_KG = 30.0
MIN_DRIVE_RPM = 50.0
MAX_DRIVE_RPM = 1200.0
MIN_INTAKE_RPM = 100.0
MAX_INTAKE_RPM = 2200.0
MIN_WHEEL_DIAMETER_IN = 1.0
MAX_WHEEL_DIAMETER_IN = 8.0
MIN_DEADZONE = 0.0
MAX_DEADZONE = 0.50
MIN_AXIS_EXPONENT = 0.40
MAX_AXIS_EXPONENT = 4.00
MIN_AXIS_INDEX = 0
MAX_AXIS_INDEX = 15


@dataclass(frozen=True)
class RobotSettings:
    width_in: float = ROBOT_SIZE_IN
    length_in: float = ROBOT_SIZE_IN
    mass_kg: float = ROBOT_DEFAULT_MASS_KG
    drivetrain_rpm: float = DRIVETRAIN_DEFAULT_RPM
    intake_rpm: float = float(INTAKE_RPM)
    wheel_diameter_in: float = DRIVE_WHEEL_DEFAULT_DIAMETER_IN
    length_unit: str = "in"
    mass_unit: str = "kg"
    outtake_from_back: bool = False

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            return cls()
        return cls(
            width_in=_num(data.get("width_in"), ROBOT_SIZE_IN, MIN_ROBOT_SIZE_IN, MAX_ROBOT_SIZE_IN),
            length_in=_num(data.get("length_in"), ROBOT_SIZE_IN, MIN_ROBOT_SIZE_IN, MAX_ROBOT_SIZE_IN),
            mass_kg=_num(data.get("mass_kg"), ROBOT_DEFAULT_MASS_KG, MIN_MASS_KG, MAX_MASS_KG),
            drivetrain_rpm=_num(data.get("drivetrain_rpm"), DRIVETRAIN_DEFAULT_RPM, MIN_DRIVE_RPM, MAX_DRIVE_RPM),
            intake_rpm=_num(data.get("intake_rpm"), float(INTAKE_RPM), MIN_INTAKE_RPM, MAX_INTAKE_RPM),
            wheel_diameter_in=_num(
                data.get("wheel_diameter_in"),
                DRIVE_WHEEL_DEFAULT_DIAMETER_IN,
                MIN_WHEEL_DIAMETER_IN,
                MAX_WHEEL_DIAMETER_IN,
            ),
            length_unit=_choice(data.get("length_unit"), LENGTH_UNITS, "in"),
            mass_unit=_choice(data.get("mass_unit"), MASS_UNITS, "kg"),
            outtake_from_back=bool(data.get("outtake_from_back", False)),
        )


@dataclass(frozen=True)
class ControllerSettings:
    forward_axis: int = VEX_FORWARD_AXIS
    turn_axis: int = VEX_TURN_AXIS
    invert_forward: bool = VEX_INVERT_FORWARD
    invert_turn: bool = VEX_INVERT_TURN
    deadzone: float = GAMEPAD_DEADZONE
    axis_exponent: float = GAMEPAD_AXIS_EXPONENT
    intake_binding: str = ""
    loader_binding: str = ""
    dump_binding: str = ""

    def __post_init__(self):
        object.__setattr__(self, "intake_binding", self.intake_binding or _buttons_binding(VEX_BUTTON_INTAKE_TOGGLE))
        object.__setattr__(self, "loader_binding", self.loader_binding or _hat_binding(VEX_LOADER_HAT_INDEX, VEX_LOADER_HAT_VECTOR))
        object.__setattr__(self, "dump_binding", self.dump_binding or _buttons_binding(VEX_BUTTON_DUMP_HOLD))

    @classmethod
    def from_dict(cls, data):
        if not isinstance(data, dict):
            return cls()
        fallback = cls()
        return cls(
            forward_axis=int(_num(data.get("forward_axis"), VEX_FORWARD_AXIS, MIN_AXIS_INDEX, MAX_AXIS_INDEX)),
            turn_axis=int(_num(data.get("turn_axis"), VEX_TURN_AXIS, MIN_AXIS_INDEX, MAX_AXIS_INDEX)),
            invert_forward=bool(data.get("invert_forward", VEX_INVERT_FORWARD)),
            invert_turn=bool(data.get("invert_turn", VEX_INVERT_TURN)),
            deadzone=_num(data.get("deadzone"), GAMEPAD_DEADZONE, MIN_DEADZONE, MAX_DEADZONE),
            axis_exponent=_num(data.get("axis_exponent"), GAMEPAD_AXIS_EXPONENT, MIN_AXIS_EXPONENT, MAX_AXIS_EXPONENT),
            intake_binding=parse_binding_text(data.get("intake_binding", ""), fallback.intake_binding),
            loader_binding=parse_binding_text(data.get("loader_binding", ""), fallback.loader_binding),
            dump_binding=parse_binding_text(data.get("dump_binding", ""), fallback.dump_binding),
        )


@dataclass(frozen=True)
class TeamSettings:
    team_number: str
    team_name: str
    robot: RobotSettings
    controller: ControllerSettings

    @classmethod
    def default(cls, team_number: str, team_name: str):
        return cls(
            team_number=team_number.strip().upper(),
            team_name=team_name.strip(),
            robot=RobotSettings(),
            controller=ControllerSettings(),
        )

    @classmethod
    def from_dict(cls, data, team_number: str, team_name: str):
        if not isinstance(data, dict):
            return cls.default(team_number, team_name)
        return cls(
            team_number=str(data.get("team_number", team_number)).strip().upper(),
            team_name=str(data.get("team_name", team_name)).strip(),
            robot=RobotSettings.from_dict(data.get("robot")),
            controller=ControllerSettings.from_dict(data.get("controller")),
        )

    def to_dict(self):
        return asdict(self)


def load_team_settings(team_number: str, team_name: str = "") -> TeamSettings:
    team_number = team_number.strip().upper()
    data = _load_all_raw()
    saved = data.get(_team_key(team_number))
    settings = TeamSettings.from_dict(saved, team_number, team_name)
    if team_name.strip() and settings.team_name != team_name.strip():
        settings = TeamSettings(
            team_number=settings.team_number,
            team_name=team_name.strip(),
            robot=settings.robot,
            controller=settings.controller,
        )
    return settings


def save_team_settings(settings: TeamSettings):
    data = _load_all_raw()
    data[_team_key(settings.team_number)] = settings.to_dict()
    try:
        TEAM_CONFIG_PATH.write_text(json.dumps(data, indent=2), encoding="utf-8")
    except OSError:
        pass


def has_saved_team_settings(team_number: str) -> bool:
    return _team_key(team_number) in _load_all_raw()


def get_saved_team_name(team_number: str) -> str:
    data = _load_all_raw().get(_team_key(team_number))
    if not isinstance(data, dict):
        return ""
    return str(data.get("team_name", "")).strip()


def inches_to_display(value_in: float, unit: str) -> float:
    return value_in * CM_PER_IN if unit == "cm" else value_in


def display_to_inches(value: float, unit: str) -> float:
    return value / CM_PER_IN if unit == "cm" else value


def kg_to_display(value_kg: float, unit: str) -> float:
    return value_kg * LB_PER_KG if unit == "lb" else value_kg


def display_to_kg(value: float, unit: str) -> float:
    return value / LB_PER_KG if unit == "lb" else value


def default_axis_for_kind(kind: str) -> int:
    if kind == "forward":
        return VEX_FORWARD_AXIS if VEX_FORWARD_AXIS >= 0 else GENERIC_FORWARD_AXIS
    return VEX_TURN_AXIS if VEX_TURN_AXIS >= 0 else GENERIC_TURN_AXIS


def default_invert_for_kind(kind: str) -> bool:
    if kind == "forward":
        return VEX_INVERT_FORWARD if VEX_FORWARD_AXIS >= 0 else GENERIC_INVERT_FORWARD
    return VEX_INVERT_TURN if VEX_TURN_AXIS >= 0 else GENERIC_INVERT_TURN


def parse_binding_text(text, fallback: str) -> str:
    raw = str(text or "").strip().lower()
    if not raw:
        return fallback

    parts = []
    for token in raw.replace("|", ",").replace(";", ",").split(","):
        token = token.strip().replace(" ", "")
        if not token:
            continue
        parsed = _parse_binding_token(token)
        if parsed is None:
            return fallback
        parts.append(parsed)

    return "|".join(parts) if parts else fallback


def binding_to_text(binding: str) -> str:
    labels = []
    for part in str(binding or "").split("|"):
        kind, values = _split_binding(part)
        if kind == "button" and values:
            labels.append(str(values[0]))
        elif kind == "hat" and len(values) >= 2:
            labels.append(f"hat{values[0]}:{values[1]}")
    return ",".join(labels)


def validate_binding_text(text: str) -> bool:
    sentinel = "__invalid__"
    return parse_binding_text(text, sentinel) != sentinel


def binding_pressed(binding: str, button_down, hat_matches) -> bool:
    for part in str(binding or "").split("|"):
        kind, values = _split_binding(part)
        if kind == "button" and values and button_down(int(values[0])):
            return True
        if kind == "hat" and len(values) >= 2 and hat_matches(int(values[0]), _direction_vector(values[1])):
            return True
    return False


def _load_all_raw():
    if not TEAM_CONFIG_PATH.exists():
        return {}
    try:
        data = json.loads(TEAM_CONFIG_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    return data if isinstance(data, dict) else {}


def _team_key(team_number: str) -> str:
    return team_number.strip().casefold()


def _num(value, default, lo, hi):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        parsed = float(default)
    return clamp(parsed, float(lo), float(hi))


def _choice(value, allowed, default):
    value = str(value or "").strip().lower()
    return value if value in allowed else default


def _buttons_binding(buttons) -> str:
    return "|".join(f"button:{int(button)}" for button in buttons)


def _hat_binding(index: int, vector) -> str:
    direction = {
        (0, 1): "up",
        (0, -1): "down",
        (-1, 0): "left",
        (1, 0): "right",
    }.get(tuple(vector), "down")
    return f"hat:{int(index)}:{direction}"


def _parse_binding_token(token: str):
    if token.startswith("button:"):
        token = token.split(":", 1)[1]
    elif token.startswith("btn"):
        token = token[3:]
    elif token.startswith("b") and token[1:].isdigit():
        token = token[1:]

    if token.isdigit():
        return f"button:{int(token)}"

    normalized = token.replace("-", ":")
    if normalized.startswith("hat"):
        normalized = normalized[3:]
        if normalized.startswith(":"):
            normalized = normalized[1:]
        if ":" in normalized:
            idx_text, direction = normalized.split(":", 1)
        else:
            digits = "".join(ch for ch in normalized if ch.isdigit())
            direction = normalized[len(digits):]
            idx_text = digits
        if idx_text.isdigit() and direction in {"up", "down", "left", "right"}:
            return f"hat:{int(idx_text)}:{direction}"

    if normalized.startswith("h") and ":" in normalized:
        idx_text, direction = normalized[1:].split(":", 1)
        if idx_text.isdigit() and direction in {"up", "down", "left", "right"}:
            return f"hat:{int(idx_text)}:{direction}"

    return None


def _split_binding(part: str):
    bits = str(part or "").split(":")
    if len(bits) >= 2 and bits[0] == "button":
        try:
            return "button", (int(bits[1]),)
        except ValueError:
            return "", ()
    if len(bits) >= 3 and bits[0] == "hat" and bits[2] in {"up", "down", "left", "right"}:
        try:
            return "hat", (int(bits[1]), bits[2])
        except ValueError:
            return "", ()
    return "", ()


def _direction_vector(direction: str):
    return {
        "up": (0, 1),
        "down": (0, -1),
        "left": (-1, 0),
        "right": (1, 0),
    }.get(direction, (0, -1))
