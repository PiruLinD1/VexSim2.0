import sys
from dataclasses import dataclass

import pygame

from config import (
    SCREEN_SIZE,
    FPS,
    MATCH_TIME_SEC,
    FIXED_DT,
    MAX_ACCUM,
    FIELD_SIZE_PX,
)

from physics import (
    create_space,
    add_field_boundaries,
    spawn_initial_blocks,
    apply_ground_friction_and_wobble,
)
from robot import Robot, RobotDriveConfig
from loaders import LoaderManager
from input_manager import InputManager
from leaderboard import load_leaderboard, add_leaderboard_entry
from team_config import (
    ControllerSettings,
    MAX_AXIS_EXPONENT,
    MAX_AXIS_INDEX,
    MAX_DEADZONE,
    MAX_DRIVE_RPM,
    MAX_INTAKE_RPM,
    MAX_MASS_KG,
    MAX_ROBOT_SIZE_IN,
    MAX_WHEEL_DIAMETER_IN,
    MIN_AXIS_EXPONENT,
    MIN_AXIS_INDEX,
    MIN_DEADZONE,
    MIN_DRIVE_RPM,
    MIN_INTAKE_RPM,
    MIN_MASS_KG,
    MIN_ROBOT_SIZE_IN,
    MIN_WHEEL_DIAMETER_IN,
    RobotSettings,
    TeamSettings,
    binding_to_text,
    display_to_inches,
    display_to_kg,
    has_saved_team_settings,
    inches_to_display,
    get_saved_team_name,
    kg_to_display,
    load_team_settings,
    parse_binding_text,
    save_team_settings,
    validate_binding_text,
)

from goals import GoalManager

from render import (
    draw_hud,
    draw_fps_indicator,
    draw_start_screen,
    draw_robot_config_screen,
    draw_quit_confirmation,
    compute_total_score,
)
from render3d import (
    DEFAULT_RENDER_QUALITY_INDEX,
    draw_match_3d,
    next_render_quality_index,
    render_quality_name,
)
from render_gpu import GpuDisplay, GPU_AVAILABLE, GPU_IMPORT_ERROR


CONFIG_FIELD_ORDER = [
    "width",
    "length",
    "mass",
    "drivetrain_rpm",
    "intake_rpm",
    "wheel_diameter",
    "forward_axis",
    "turn_axis",
    "deadzone",
    "axis_exponent",
    "intake_binding",
    "loader_binding",
    "dump_binding",
]

BINDING_FIELDS = {"intake_binding", "loader_binding", "dump_binding"}
AXIS_FIELDS = {"forward_axis", "turn_axis"}


@dataclass
class SessionProfile:
    team_number: str
    team_name: str
    is_guest: bool = False

    @property
    def display_name(self):
        if self.is_guest:
            return "Guest"
        return f"{self.team_number} {self.team_name}".strip()


@dataclass
class MatchSession:
    space: object
    goals: GoalManager
    loaders: LoaderManager
    robot: Robot
    time_left: float = MATCH_TIME_SEC
    accum: float = 0.0


def create_match_session(team_settings: TeamSettings | None = None):
    space = create_space()
    add_field_boundaries(space)

    goals = GoalManager()
    goals.build(space)

    loaders = LoaderManager()
    if hasattr(goals, "long_goals") and len(goals.long_goals) >= 2:
        loaders.build(
            [
                {"cx": goals.long_goals[0].cx},
                {"cx": goals.long_goals[1].cx},
            ]
        )
    else:
        raise RuntimeError("GoalManager did not expose long_goals as expected.")
    loaders.build_physics(space)

    spawn_initial_blocks(space)

    robot_settings = team_settings.robot if team_settings is not None else RobotSettings()
    track_width_in = max(1.0, robot_settings.width_in * (10.5 / 12.0))

    robot = Robot(
        space,
        (FIELD_SIZE_PX * 0.25, FIELD_SIZE_PX * 0.80),
        drive_config=RobotDriveConfig(
            width_in=robot_settings.width_in,
            length_in=robot_settings.length_in,
            mass_kg=robot_settings.mass_kg,
            drivetrain_rpm=robot_settings.drivetrain_rpm,
            intake_rpm=robot_settings.intake_rpm,
            wheel_diameter_in=robot_settings.wheel_diameter_in,
            outtake_from_back=robot_settings.outtake_from_back,
            track_width_in=track_width_in,
        ),
    )
    return MatchSession(space=space, goals=goals, loaders=loaders, robot=robot)


def append_text(current, incoming, max_len, uppercase=False):
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 -_"
    out = current
    for ch in incoming:
        if ch not in allowed:
            continue
        if len(out) >= max_len:
            break
        out += ch.upper() if uppercase else ch
    return out


def append_config_text(current, incoming, max_len):
    allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 .,:;_|-+"
    out = current
    for ch in incoming:
        if ch not in allowed:
            continue
        if len(out) >= max_len:
            break
        out += ch
    return out


def settings_to_draft(settings: TeamSettings):
    robot = settings.robot
    controller = settings.controller
    length_unit = robot.length_unit
    mass_unit = robot.mass_unit
    return {
        "length_unit": length_unit,
        "mass_unit": mass_unit,
        "outtake": "back" if robot.outtake_from_back else "front",
        "invert_forward": controller.invert_forward,
        "invert_turn": controller.invert_turn,
        "values": {
            "width": _fmt(inches_to_display(robot.width_in, length_unit)),
            "length": _fmt(inches_to_display(robot.length_in, length_unit)),
            "mass": _fmt(kg_to_display(robot.mass_kg, mass_unit)),
            "drivetrain_rpm": _fmt(robot.drivetrain_rpm, 0),
            "intake_rpm": _fmt(robot.intake_rpm, 0),
            "wheel_diameter": _fmt(inches_to_display(robot.wheel_diameter_in, length_unit)),
            "forward_axis": str(controller.forward_axis),
            "turn_axis": str(controller.turn_axis),
            "deadzone": _fmt(controller.deadzone, 2),
            "axis_exponent": _fmt(controller.axis_exponent, 2),
            "intake_binding": binding_to_text(controller.intake_binding),
            "loader_binding": binding_to_text(controller.loader_binding),
            "dump_binding": binding_to_text(controller.dump_binding),
        },
    }


def draft_to_settings(profile: SessionProfile, draft):
    values = draft.get("values", {})
    length_unit = draft.get("length_unit", "in")
    mass_unit = draft.get("mass_unit", "kg")

    try:
        width_in = display_to_inches(_float_field(values, "width", "width"), length_unit)
        length_in = display_to_inches(_float_field(values, "length", "length"), length_unit)
        mass_kg = display_to_kg(_float_field(values, "mass", "weight"), mass_unit)
        drivetrain_rpm = _float_field(values, "drivetrain_rpm", "drive RPM")
        intake_rpm = _float_field(values, "intake_rpm", "intake RPM")
        wheel_diameter_in = display_to_inches(_float_field(values, "wheel_diameter", "drive wheel"), length_unit)
        forward_axis = _int_field(values, "forward_axis", "forward axis")
        turn_axis = _int_field(values, "turn_axis", "turn axis")
        deadzone = _float_field(values, "deadzone", "deadzone")
        axis_exponent = _float_field(values, "axis_exponent", "stick curve")
    except ValueError as exc:
        return None, str(exc)

    checks = [
        (MIN_ROBOT_SIZE_IN <= width_in <= MAX_ROBOT_SIZE_IN, f"Width must be {MIN_ROBOT_SIZE_IN:g}-{MAX_ROBOT_SIZE_IN:g} in."),
        (MIN_ROBOT_SIZE_IN <= length_in <= MAX_ROBOT_SIZE_IN, f"Length must be {MIN_ROBOT_SIZE_IN:g}-{MAX_ROBOT_SIZE_IN:g} in."),
        (MIN_MASS_KG <= mass_kg <= MAX_MASS_KG, f"Weight must be {MIN_MASS_KG:g}-{MAX_MASS_KG:g} kg."),
        (MIN_DRIVE_RPM <= drivetrain_rpm <= MAX_DRIVE_RPM, f"Drive RPM must be {MIN_DRIVE_RPM:g}-{MAX_DRIVE_RPM:g}."),
        (MIN_INTAKE_RPM <= intake_rpm <= MAX_INTAKE_RPM, f"Intake RPM must be {MIN_INTAKE_RPM:g}-{MAX_INTAKE_RPM:g}."),
        (MIN_WHEEL_DIAMETER_IN <= wheel_diameter_in <= MAX_WHEEL_DIAMETER_IN, f"Wheel diameter must be {MIN_WHEEL_DIAMETER_IN:g}-{MAX_WHEEL_DIAMETER_IN:g} in."),
        (MIN_AXIS_INDEX <= forward_axis <= MAX_AXIS_INDEX, f"Forward axis must be {MIN_AXIS_INDEX}-{MAX_AXIS_INDEX}."),
        (MIN_AXIS_INDEX <= turn_axis <= MAX_AXIS_INDEX, f"Turn axis must be {MIN_AXIS_INDEX}-{MAX_AXIS_INDEX}."),
        (MIN_DEADZONE <= deadzone <= MAX_DEADZONE, f"Deadzone must be {MIN_DEADZONE:g}-{MAX_DEADZONE:g}."),
        (MIN_AXIS_EXPONENT <= axis_exponent <= MAX_AXIS_EXPONENT, f"Stick curve must be {MIN_AXIS_EXPONENT:g}-{MAX_AXIS_EXPONENT:g}."),
    ]
    for ok, message in checks:
        if not ok:
            return None, message

    default_controller = ControllerSettings()
    for key, label in (
        ("intake_binding", "intake button"),
        ("loader_binding", "loader button"),
        ("dump_binding", "outtake button"),
    ):
        if not validate_binding_text(values.get(key, "")):
            return None, f"Invalid {label}. Use a button number or hat0:down."

    robot = RobotSettings(
        width_in=width_in,
        length_in=length_in,
        mass_kg=mass_kg,
        drivetrain_rpm=drivetrain_rpm,
        intake_rpm=intake_rpm,
        wheel_diameter_in=wheel_diameter_in,
        length_unit=length_unit,
        mass_unit=mass_unit,
        outtake_from_back=draft.get("outtake") == "back",
    )
    controller = ControllerSettings(
        forward_axis=forward_axis,
        turn_axis=turn_axis,
        invert_forward=bool(draft.get("invert_forward", False)),
        invert_turn=bool(draft.get("invert_turn", False)),
        deadzone=deadzone,
        axis_exponent=axis_exponent,
        intake_binding=parse_binding_text(values.get("intake_binding", ""), default_controller.intake_binding),
        loader_binding=parse_binding_text(values.get("loader_binding", ""), default_controller.loader_binding),
        dump_binding=parse_binding_text(values.get("dump_binding", ""), default_controller.dump_binding),
    )
    return TeamSettings(profile.team_number, profile.team_name, robot, controller), ""


def switch_length_unit(draft, new_unit):
    old_unit = draft.get("length_unit", "in")
    if new_unit == old_unit:
        return
    values = draft.get("values", {})
    for key in ("width", "length", "wheel_diameter"):
        try:
            current_in = display_to_inches(_parse_float(values.get(key, "")), old_unit)
        except ValueError:
            continue
        values[key] = _fmt(inches_to_display(current_in, new_unit))
    draft["length_unit"] = new_unit


def switch_mass_unit(draft, new_unit):
    old_unit = draft.get("mass_unit", "kg")
    if new_unit == old_unit:
        return
    values = draft.get("values", {})
    try:
        current_kg = display_to_kg(_parse_float(values.get("mass", "")), old_unit)
    except ValueError:
        draft["mass_unit"] = new_unit
        return
    values["mass"] = _fmt(kg_to_display(current_kg, new_unit))
    draft["mass_unit"] = new_unit


def _fmt(value, decimals=2):
    text = f"{float(value):.{decimals}f}"
    if "." in text:
        text = text.rstrip("0").rstrip(".")
    return text


def _parse_float(value):
    text = str(value).strip().replace(",", ".")
    if text == "":
        raise ValueError
    return float(text)


def _float_field(values, key, label):
    try:
        return _parse_float(values.get(key, ""))
    except (TypeError, ValueError):
        raise ValueError(f"Invalid {label}.")


def _int_field(values, key, label):
    try:
        return int(round(_parse_float(values.get(key, ""))))
    except (TypeError, ValueError):
        raise ValueError(f"Invalid {label}.")


def start_ranked_run(team_number, team_name):
    number = team_number.strip().upper()
    name = team_name.strip()
    if not number or not name:
        return None
    return SessionProfile(team_number=number, team_name=name, is_guest=False)


def start_guest_run():
    return SessionProfile(team_number="GUEST", team_name="Guest", is_guest=True)


def begin_match(profile, input_manager, team_settings: TeamSettings | None = None):
    match = create_match_session(team_settings)
    input_manager.set_controller_settings(None if profile.is_guest or team_settings is None else team_settings.controller)
    pygame.key.stop_text_input()
    return profile, match, "match", ""


def prepare_config_for_profile(profile: SessionProfile):
    saved = has_saved_team_settings(profile.team_number)
    settings = load_team_settings(profile.team_number, profile.team_name)
    draft = settings_to_draft(settings)
    if saved:
        status = "Saved robot profile loaded."
    else:
        status = "New team profile. Tune it!"
    return settings, draft, status


def lookup_saved_team_name(team_number: str, leaderboard_entries):
    number = team_number.strip()
    if not number:
        return ""

    saved_name = get_saved_team_name(number)
    if saved_name:
        return saved_name

    key = number.casefold()
    for entry in leaderboard_entries:
        if str(entry.get("team_number", "")).strip().casefold() == key:
            return str(entry.get("team_name", "")).strip()
    return ""


def validate_and_save_profile(profile: SessionProfile, draft):
    settings, error = draft_to_settings(profile, draft)
    if settings is None:
        return None, error
    save_team_settings(settings)
    return settings, ""


def controller_binding_from_event(event):
    if event.type == pygame.JOYBUTTONDOWN:
        return str(int(getattr(event, "button", 0)))
    if event.type == pygame.JOYHATMOTION:
        value = tuple(getattr(event, "value", (0, 0)))
        direction = {
            (0, 1): "up",
            (0, -1): "down",
            (-1, 0): "left",
            (1, 0): "right",
        }.get(value)
        if direction is not None:
            return f"hat{int(getattr(event, 'hat', 0))}:{direction}"
    return None


def controller_axis_from_event(event):
    if event.type != pygame.JOYAXISMOTION:
        return None
    if abs(float(getattr(event, "value", 0.0))) < 0.65:
        return None
    return str(int(getattr(event, "axis", 0)))


def config_capture_message(field_name: str):
    if field_name in BINDING_FIELDS:
        return "Press a controller button or D-pad direction for this action."
    if field_name in AXIS_FIELDS:
        return "Move the stick you want to use for this axis."
    return ""


def main():
    pygame.init()
    pygame.key.start_text_input()

    input_manager = InputManager()

    gpu_display = None
    gpu_startup_message = ""
    try:
        if not GPU_AVAILABLE:
            raise RuntimeError(
                f"PyOpenGL is missing ({GPU_IMPORT_ERROR}). "
                "Install it with: python -m pip install PyOpenGL"
            )
        gpu_display = GpuDisplay(SCREEN_SIZE)
        window = None
    except Exception as exc:
        gpu_display = None
        window = pygame.display.set_mode(SCREEN_SIZE, pygame.FULLSCREEN | pygame.SCALED)
        gpu_startup_message = f"GPU renderer unavailable: {str(exc).splitlines()[0]}"
    pygame.display.set_caption("VEX Push Back - Goals + Loaders")

    if gpu_display is None:
        canvas = pygame.Surface(SCREEN_SIZE).convert()
    else:
        canvas = pygame.Surface(SCREEN_SIZE, pygame.SRCALPHA).convert_alpha()

    clock = pygame.time.Clock()
    font_small = pygame.font.SysFont("consolas", 18)
    font_big = pygame.font.SysFont("consolas", 26, bold=True)
    font_title = pygame.font.SysFont("consolas", 42, bold=True)

    leaderboard_entries = load_leaderboard()

    team_number = ""
    team_name = ""
    active_field = "team_number"
    status_message = gpu_startup_message

    app_state = "menu"
    current_profile = None
    current_settings = None
    match = None
    menu_hitboxes = None
    config_hitboxes = None
    config_draft = None
    active_config_field = CONFIG_FIELD_ORDER[0]
    quit_confirm_hitboxes = None
    confirm_quit = False
    next_team_autofill_ms = 0
    last_autofill_number = ""
    last_autofill_name = ""
    render_quality_index = DEFAULT_RENDER_QUALITY_INDEX

    running = True

    while running:
        gpu_presented = False
        dt = clock.tick(FPS) / 1000.0
        fps_now = clock.get_fps()
        if dt <= 0.0:
            dt = 1.0 / FPS
        if dt > 0.05:
            dt = 0.05

        events = pygame.event.get()
        for event in events:
            input_manager.handle_event(event)
            if event.type == pygame.QUIT:
                running = False

        if not running:
            break

        if app_state == "menu":
            if confirm_quit:
                for event in events:
                    if event.type == pygame.KEYDOWN:
                        if event.key in (pygame.K_RETURN, pygame.K_KP_ENTER, pygame.K_y):
                            running = False
                            break
                        if event.key in (pygame.K_ESCAPE, pygame.K_n):
                            confirm_quit = False
                    elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and quit_confirm_hitboxes is not None:
                        if quit_confirm_hitboxes["yes"].collidepoint(event.pos):
                            running = False
                            break
                        if quit_confirm_hitboxes["no"].collidepoint(event.pos):
                            confirm_quit = False

                if not running:
                    break

                menu_hitboxes = draw_start_screen(
                    canvas,
                    font_small,
                    font_big,
                    font_title,
                    team_number,
                    team_name,
                    active_field,
                    leaderboard_entries,
                    status_message,
                )
                if confirm_quit:
                    quit_confirm_hitboxes = draw_quit_confirmation(canvas, font_small, font_big)
                draw_fps_indicator(canvas, font_small, fps_now)
            else:
                for event in events:
                    if app_state != "menu":
                        break
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            confirm_quit = True
                        elif event.key == pygame.K_TAB:
                            active_field = "team_name" if active_field == "team_number" else "team_number"
                        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                            profile = start_ranked_run(team_number, team_name)
                            if profile is None:
                                status_message = "Enter both team number and team name before starting."
                            else:
                                current_profile = profile
                                current_settings, config_draft, status_message = prepare_config_for_profile(profile)
                                active_config_field = CONFIG_FIELD_ORDER[0]
                                app_state = "config"
                                pygame.key.start_text_input()
                        elif event.key == pygame.K_g and active_field != "team_name":
                            profile = start_guest_run()
                            current_profile, match, app_state, _ = begin_match(profile, input_manager, None)
                            current_settings = None
                            status_message = "Guest run started. Score will not be saved."
                        elif event.key == pygame.K_BACKSPACE:
                            if active_field == "team_number":
                                team_number = team_number[:-1]
                            else:
                                team_name = team_name[:-1]
                    elif event.type == pygame.TEXTINPUT:
                        if active_field == "team_number":
                            team_number = append_text(team_number, event.text, 12, uppercase=True)
                        else:
                            team_name = append_text(team_name, event.text, 24, uppercase=False)
                    elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and menu_hitboxes is not None:
                        mouse_pos = event.pos
                        if menu_hitboxes["team_number"].collidepoint(mouse_pos):
                            active_field = "team_number"
                        elif menu_hitboxes["team_name"].collidepoint(mouse_pos):
                            active_field = "team_name"
                        else:
                            buttons = menu_hitboxes.get("buttons", {})
                            if buttons.get("ranked") and buttons["ranked"].collidepoint(mouse_pos):
                                profile = start_ranked_run(team_number, team_name)
                                if profile is None:
                                    status_message = "Enter both team number and team name before starting."
                                else:
                                    current_profile = profile
                                    current_settings, config_draft, status_message = prepare_config_for_profile(profile)
                                    active_config_field = CONFIG_FIELD_ORDER[0]
                                    app_state = "config"
                                    pygame.key.start_text_input()
                            elif buttons.get("guest") and buttons["guest"].collidepoint(mouse_pos):
                                profile = start_guest_run()
                                current_profile, match, app_state, _ = begin_match(profile, input_manager, None)
                                current_settings = None
                                status_message = "Guest run started. Score will not be saved."
                            elif buttons.get("quit") and buttons["quit"].collidepoint(mouse_pos):
                                confirm_quit = True

                if app_state == "menu":
                    now_ms = pygame.time.get_ticks()
                    if now_ms >= next_team_autofill_ms:
                        next_team_autofill_ms = now_ms + 500
                        current_number = team_number.strip().upper()
                        if current_number:
                            found_name = lookup_saved_team_name(current_number, leaderboard_entries)
                            number_changed = current_number != last_autofill_number
                            if found_name:
                                can_overwrite_name = (
                                    number_changed
                                    or not team_name.strip()
                                    or team_name == last_autofill_name
                                )
                                if can_overwrite_name and team_name != found_name:
                                    team_name = found_name
                                    status_message = f"Loaded team name for {current_number}."
                                last_autofill_number = current_number
                                last_autofill_name = found_name
                            else:
                                if number_changed and team_name == last_autofill_name:
                                    team_name = ""
                                last_autofill_number = current_number
                                last_autofill_name = ""
                        else:
                            last_autofill_number = ""
                            last_autofill_name = ""

                    menu_hitboxes = draw_start_screen(
                        canvas,
                        font_small,
                        font_big,
                        font_title,
                        team_number,
                        team_name,
                        active_field,
                        leaderboard_entries,
                        status_message,
                    )
                    draw_fps_indicator(canvas, font_small, fps_now)

        elif app_state == "config":
            if current_profile is None or config_draft is None:
                app_state = "menu"
                status_message = "No team profile is open."
            else:
                values = config_draft.setdefault("values", {})
                for event in events:
                    if app_state != "config":
                        break
                    if event.type == pygame.KEYDOWN:
                        if event.key == pygame.K_ESCAPE:
                            app_state = "menu"
                            status_message = "Robot setup cancelled."
                            current_profile = None
                            current_settings = None
                            config_draft = None
                        elif event.key == pygame.K_TAB:
                            idx = CONFIG_FIELD_ORDER.index(active_config_field)
                            active_config_field = CONFIG_FIELD_ORDER[(idx + 1) % len(CONFIG_FIELD_ORDER)]
                            capture_message = config_capture_message(active_config_field)
                            if capture_message:
                                status_message = capture_message
                        elif event.key in (pygame.K_RETURN, pygame.K_KP_ENTER):
                            saved_settings, error = validate_and_save_profile(current_profile, config_draft)
                            if saved_settings is None:
                                status_message = error
                            else:
                                current_settings = saved_settings
                                current_profile, match, app_state, status_message = begin_match(
                                    current_profile,
                                    input_manager,
                                    current_settings,
                                )
                        elif event.key == pygame.K_BACKSPACE:
                            if active_config_field in BINDING_FIELDS:
                                values[active_config_field] = ""
                                status_message = config_capture_message(active_config_field)
                            else:
                                values[active_config_field] = values.get(active_config_field, "")[:-1]
                    elif event.type in (pygame.JOYBUTTONDOWN, pygame.JOYHATMOTION) and active_config_field in BINDING_FIELDS:
                        captured = controller_binding_from_event(event)
                        if captured is not None:
                            values[active_config_field] = captured
                            status_message = f"Captured {captured} for {active_config_field.replace('_', ' ')}."
                    elif event.type == pygame.JOYAXISMOTION and active_config_field in AXIS_FIELDS:
                        captured_axis = controller_axis_from_event(event)
                        if captured_axis is not None:
                            values[active_config_field] = captured_axis
                            status_message = f"Captured axis {captured_axis} for {active_config_field.replace('_', ' ')}."
                    elif event.type == pygame.TEXTINPUT:
                        if active_config_field in BINDING_FIELDS:
                            status_message = config_capture_message(active_config_field)
                        else:
                            max_len = 12
                            values[active_config_field] = append_config_text(
                                values.get(active_config_field, ""),
                                event.text,
                                max_len,
                            )
                    elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and config_hitboxes is not None:
                        mouse_pos = event.pos
                        for field_name, rect in config_hitboxes.get("fields", {}).items():
                            if rect.collidepoint(mouse_pos):
                                active_config_field = field_name
                                capture_message = config_capture_message(active_config_field)
                                if capture_message:
                                    status_message = capture_message
                                break
                        else:
                            clicked = False
                            for key, rect in config_hitboxes.get("toggles", {}).items():
                                if not rect.collidepoint(mouse_pos):
                                    continue
                                clicked = True
                                if key.startswith("length_unit:"):
                                    switch_length_unit(config_draft, key.split(":", 1)[1])
                                elif key.startswith("mass_unit:"):
                                    switch_mass_unit(config_draft, key.split(":", 1)[1])
                                elif key.startswith("outtake:"):
                                    config_draft["outtake"] = key.split(":", 1)[1]
                                elif key == "invert_forward":
                                    config_draft["invert_forward"] = not bool(config_draft.get("invert_forward", False))
                                elif key == "invert_turn":
                                    config_draft["invert_turn"] = not bool(config_draft.get("invert_turn", False))
                                break
                            if clicked:
                                continue

                            buttons = config_hitboxes.get("buttons", {})
                            if buttons.get("back") and buttons["back"].collidepoint(mouse_pos):
                                app_state = "menu"
                                status_message = "Robot setup cancelled."
                                current_profile = None
                                current_settings = None
                                config_draft = None
                            elif buttons.get("save") and buttons["save"].collidepoint(mouse_pos):
                                saved_settings, error = validate_and_save_profile(current_profile, config_draft)
                                if saved_settings is None:
                                    status_message = error
                                else:
                                    current_settings = saved_settings
                                    status_message = "Robot and controller profile saved."
                            elif buttons.get("start") and buttons["start"].collidepoint(mouse_pos):
                                saved_settings, error = validate_and_save_profile(current_profile, config_draft)
                                if saved_settings is None:
                                    status_message = error
                                else:
                                    current_settings = saved_settings
                                    current_profile, match, app_state, status_message = begin_match(
                                        current_profile,
                                        input_manager,
                                        current_settings,
                                    )

            if app_state == "config":
                controller_name = "Keyboard"
                if input_manager.active_joystick is not None:
                    controller_name = input_manager.active_joystick.get_name()
                config_hitboxes = draw_robot_config_screen(
                    canvas,
                    font_small,
                    font_big,
                    font_title,
                    current_profile,
                    config_draft,
                    active_config_field,
                    status_message,
                    controller_name,
                )
                draw_fps_indicator(canvas, font_small, fps_now)
            elif app_state == "menu":
                pygame.key.start_text_input()
                menu_hitboxes = draw_start_screen(
                    canvas,
                    font_small,
                    font_big,
                    font_title,
                    team_number,
                    team_name,
                    active_field,
                    leaderboard_entries,
                    status_message,
                )
                draw_fps_indicator(canvas, font_small, fps_now)

        elif app_state == "match":
            for event in events:
                if event.type != pygame.KEYDOWN:
                    continue
                if event.key == pygame.K_F4:
                    render_quality_index = next_render_quality_index(render_quality_index)
                    renderer_name = "GPU" if gpu_display is not None else "3D"
                    status_message = f"{renderer_name} quality: {render_quality_name(render_quality_index)}."

            escape_to_menu = any(
                event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE
                for event in events
            )
            if escape_to_menu:
                status_message = "Match cancelled. Back to menu."
                if current_profile is not None and not current_profile.is_guest:
                    team_number = current_profile.team_number
                    team_name = current_profile.team_name
                current_profile = None
                current_settings = None
                match = None
                app_state = "menu"
                input_manager.set_controller_settings(None)
                pygame.key.start_text_input()
                menu_hitboxes = draw_start_screen(
                    canvas,
                    font_small,
                    font_big,
                    font_title,
                    team_number,
                    team_name,
                    active_field,
                    leaderboard_entries,
                    status_message,
                )
                draw_fps_indicator(canvas, font_small, fps_now)
            else:
                keys = pygame.key.get_pressed()
                controls = input_manager.poll(keys)

                if controls.dump_toggle:
                    match.robot.toggle_dump()
                match.robot.set_dump_hold(controls.dump_hold)

                match.robot.set_input(
                    controls.forward,
                    controls.turn,
                    controls.intake_enabled,
                    controls.loader_active,
                )

                match.accum += dt
                if match.accum > MAX_ACCUM:
                    match.accum = MAX_ACCUM

                while match.accum >= FIXED_DT:
                    if match.time_left > 0.0:
                        match.robot.update(FIXED_DT, match.space, match.goals, match.loaders)
                        apply_ground_friction_and_wobble(match.space, FIXED_DT)
                        match.space.step(FIXED_DT)
                        match.goals.update(FIXED_DT, match.space, robot=match.robot)

                        match.time_left -= FIXED_DT
                        if match.time_left < 0.0:
                            match.time_left = 0.0
                    match.accum -= FIXED_DT

                if match.time_left <= 0.0:
                    final_score = compute_total_score(match.goals, match.loaders, match.space, match.robot)
                    if current_profile is not None and not current_profile.is_guest:
                        leaderboard_entries = add_leaderboard_entry(
                            current_profile.team_number,
                            current_profile.team_name,
                            final_score,
                        )
                        team_number = current_profile.team_number
                        team_name = current_profile.team_name
                        status_message = f"{current_profile.display_name} finished with {final_score} points. Leaderboard updated."
                    else:
                        status_message = f"Guest run finished with {final_score} points. Score not saved."

                    current_profile = None
                    current_settings = None
                    match = None
                    app_state = "menu"
                    input_manager.set_controller_settings(None)
                    pygame.key.start_text_input()

                    menu_hitboxes = draw_start_screen(
                        canvas,
                        font_small,
                        font_big,
                        font_title,
                        team_number,
                        team_name,
                        active_field,
                        leaderboard_entries,
                        status_message,
                    )
                    draw_fps_indicator(canvas, font_small, fps_now)
                else:
                    if gpu_display is None:
                        draw_match_3d(
                            canvas,
                            match.space,
                            match.goals,
                            match.loaders,
                            match.robot,
                            quality_index=render_quality_index,
                        )
                        draw_hud(
                            canvas,
                            font_small,
                            font_big,
                            match.robot,
                            match.goals,
                            match.time_left,
                            match.loaders,
                            match.space,
                            controls,
                        )
                        draw_fps_indicator(canvas, font_small, fps_now)
                    else:
                        canvas.fill((0, 0, 0, 0))
                        gpu_display.draw_match(
                            match.space,
                            match.goals,
                            match.loaders,
                            match.robot,
                            quality_index=render_quality_index,
                        )
                        draw_hud(
                            canvas,
                            font_small,
                            font_big,
                            match.robot,
                            match.goals,
                            match.time_left,
                            match.loaders,
                            match.space,
                            controls,
                        )
                        draw_fps_indicator(canvas, font_small, fps_now)
                        gpu_display.present_overlay_regions(
                            canvas,
                            (
                                pygame.Rect(FIELD_SIZE_PX, 0, SCREEN_SIZE[0] - FIELD_SIZE_PX, SCREEN_SIZE[1]),
                                pygame.Rect(10, 10, 96, 30),
                            ),
                        )
                        gpu_presented = True

        if gpu_display is None:
            window.blit(canvas, (0, 0))
            pygame.display.flip()
        elif not gpu_presented:
            gpu_display.present_canvas(canvas)

    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
