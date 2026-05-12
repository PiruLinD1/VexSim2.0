import pygame

from config import (
    FIELD_SIZE_PX,
    SCREEN_SIZE,
    INTAKE_CAPACITY,
    PARK_ZONE_OUTER_SIZE_IN,
    inches_to_px,
)

# colori
COLOR_BG = (20, 20, 25)
COLOR_TEXT = (240, 240, 240)
COLOR_RED = (255, 80, 80)
COLOR_PANEL = (28, 28, 34)
COLOR_PANEL_ALT = (36, 38, 46)
COLOR_ACCENT = (115, 190, 255)
COLOR_INPUT_ACTIVE = (255, 210, 110)
COLOR_INPUT_IDLE = (100, 105, 120)
COLOR_SUCCESS = (130, 235, 150)
COLOR_WARN = (255, 205, 130)


def draw_fps_indicator(surf, font_small, fps: float):
    fps_rect = pygame.Rect(10, 10, 96, 30)
    pygame.draw.rect(surf, (18, 18, 22), fps_rect, border_radius=8)
    pygame.draw.rect(surf, (75, 80, 92), fps_rect, 1, border_radius=8)
    color = COLOR_SUCCESS if fps >= 55.0 else COLOR_WARN if fps >= 40.0 else COLOR_RED
    label = font_small.render(f"FPS {fps:5.1f}", True, color)
    surf.blit(label, (fps_rect.x + 10, fps_rect.y + 5))


def _parking_zone_rects():
    outer = inches_to_px(PARK_ZONE_OUTER_SIZE_IN)
    half = outer * 0.5
    return {
        "blue": pygame.Rect(
            int((FIELD_SIZE_PX * 0.5) - half),
            0,
            int(outer),
            int(outer),
        ),
        "red": pygame.Rect(
            int((FIELD_SIZE_PX * 0.5) - half),
            int(FIELD_SIZE_PX - outer),
            int(outer),
            int(outer),
        ),
    }


def _point_in_rect(point, rect: pygame.Rect):
    return rect.collidepoint(float(point.x), float(point.y))


def compute_score_breakdown(goals_manager, loaders_manager=None, space=None, robot=None):
    long_goals = list(getattr(goals_manager, "long_goals", []))
    middle_goals = list(getattr(goals_manager, "middle_goals", []))
    all_goal_balls = [
        ball
        for goal in [*long_goals, *middle_goals]
        for ball in getattr(goal, "queue", [])
    ]

    scored_balls = len(all_goal_balls)
    long_center_bonus_count = 0
    for goal in long_goals:
        center_count = sum(
            1
            for ball in getattr(goal, "queue", [])
            if getattr(goal, "control_start_s", 0.0) <= ball.s <= getattr(goal, "control_end_s", 1.0)
        )
        if center_count >= 3:
            long_center_bonus_count += 1

    full_middle_count = sum(1 for goal in middle_goals if len(getattr(goal, "queue", [])) >= 7)
    empty_loader_count = 0
    if loaders_manager is not None:
        empty_loader_count = sum(1 for loader in getattr(loaders_manager, "loaders", []) if len(loader.queue) == 0)

    empty_parking_count = 0
    parked_red = False
    parking_rects = _parking_zone_rects()
    if space is not None:
        for rect in parking_rects.values():
            has_block = any(_point_in_rect(shape.body.position, rect) for shape in getattr(space, "block_shapes", []))
            if not has_block:
                empty_parking_count += 1

    if robot is not None:
        parked_red = _point_in_rect(robot.body.position, parking_rects["red"])

    bonuses = {
        "long_center": long_center_bonus_count * 5,
        "empty_loaders": empty_loader_count * 5,
        "full_middle": full_middle_count * 10,
        "empty_parking": empty_parking_count * 5,
        "red_parked": 15 if parked_red else 0,
    }
    return {
        "balls": scored_balls,
        "ball_points": scored_balls,
        "long_center_bonus_count": long_center_bonus_count,
        "empty_loader_count": empty_loader_count,
        "full_middle_count": full_middle_count,
        "empty_parking_count": empty_parking_count,
        "parked_red": parked_red,
        "bonuses": bonuses,
        "bonus_points": sum(bonuses.values()),
        "total": scored_balls + sum(bonuses.values()),
    }


def compute_total_score(goals_manager, loaders_manager=None, space=None, robot=None):
    return compute_score_breakdown(goals_manager, loaders_manager, space, robot)["total"]


def draw_hud(surf, font_small, font_big, robot, goals_manager, time_left, loaders_manager=None, space=None, controls=None):
    hud_rect = pygame.Rect(FIELD_SIZE_PX, 0, SCREEN_SIZE[0] - FIELD_SIZE_PX, SCREEN_SIZE[1])
    pygame.draw.rect(surf, (25, 25, 25), hud_rect)

    minutes = int(time_left) // 60
    seconds = int(time_left) % 60
    time_str = f"{minutes:01d}:{seconds:02d}"
    t_surf = font_big.render(f"TIME {time_str}", True, COLOR_TEXT)
    surf.blit(t_surf, (FIELD_SIZE_PX + 20, 10))

    score = compute_score_breakdown(goals_manager, loaders_manager, space, robot)
    total_score = score["total"]
    y = 60
    surf.blit(font_big.render(f"SCORE {total_score}", True, COLOR_ACCENT), (FIELD_SIZE_PX + 20, y))

    y += 60
    order_str = ",".join("R" if c == "red" else "B" for c in robot.storage)
    if len(order_str) > 30:
        order_str = order_str[:30] + "..."

    drive = robot.drive
    dbg = robot.drive_debug
    raw_buttons = "-"
    if controls and controls.pressed_buttons:
        raw_buttons = ",".join(str(b) for b in controls.pressed_buttons)
    raw_hats = "-"
    if controls and controls.hat_state:
        raw_hats = " | ".join(f"{i}:{v}" for i, v in enumerate(controls.hat_state))

    lines = [
        f"On board: {len(robot.storage)}/{INTAKE_CAPACITY}",
        f"Ordine: {order_str if order_str else '-'}",
        f"Score balls: {score['ball_points']}",
        f"Score bonus: {score['bonus_points']}",
        f"  Long center: {score['long_center_bonus_count']}x5",
        f"  Empty loaders: {score['empty_loader_count']}x5",
        f"  Full middle: {score['full_middle_count']}x10",
        f"  Empty parks: {score['empty_parking_count']}x5",
        f"  Red parked: {'15' if score['parked_red'] else '0'}",
        "",
        "Robot:",
        f"  Size: {drive.width_in:.1f} x {drive.length_in:.1f} in",
        f"  Mass: {drive.mass_kg:.1f} kg",
        f"  Drive: {drive.motor_count}x{drive.motor_power_w:.0f}W, {drive.drivetrain_rpm:.0f} rpm",
        f"  Intake: {drive.intake_rpm:.0f} rpm, out {'back' if drive.outtake_from_back else 'front'}",
        f"  Wheel: {drive.wheel_diameter_in:.1f} in, track {drive.track_width_in:.1f} in",
        f"  Free speed: {drive.wheel_free_speed_mps:.2f} m/s",
        "",
        "Live drive:",
        f"  Speed: {dbg.get('speed_mps', 0.0):.2f} m/s",
        f"  Fwd slip: {dbg.get('forward_speed_mps', 0.0):.2f} / {dbg.get('lateral_speed_mps', 0.0):.2f} m/s",
        f"  Yaw rate: {dbg.get('yaw_rate_deg_s', 0.0):.0f} deg/s",
        "",
        "Input:",
        f"  Controller: {controls.controller_name if controls else 'Keyboard'}",
        f"  Profile: {controls.controller_profile if controls else 'keyboard'}",
        f"  DZ/curve: {controls.deadzone:.2f}/{controls.axis_exponent:.2f}" if controls else "  DZ/curve: -",
        f"  Raw buttons: {raw_buttons}",
        f"  Raw hats: {raw_hats}",
        f"  Intake: {'ON' if (controls and controls.intake_enabled) else 'OFF'}",
        f"  Loader: {'ON' if (controls and controls.loader_active) else 'OFF'}",
        "  Sticks/buttons use team profile",
        "  Raw input shown for remapping",
        "",
        "Controls:",
        "  W/S, A/D : move/turn",
        "  SHIFT    : intake (floor)",
        "  E (hold) : loader paddle",
        "  SPACE    : toggle dump",
        "  F4       : 3D quality",
        "  ESC      : back to menu",
        "",
        "Dump:",
        "  - in zona gialla -> goal",
        "  - fuori zona     -> floor",
    ]
    for text in lines:
        surf.blit(font_small.render(text, True, COLOR_TEXT), (FIELD_SIZE_PX + 20, y))
        y += 22


def draw_start_screen(
    surf,
    font_small,
    font_big,
    font_title,
    team_number,
    team_name,
    active_field,
    leaderboard_entries,
    status_message="",
):
    surf.fill(COLOR_BG)

    left_rect = pygame.Rect(28, 28, int(SCREEN_SIZE[0] * 0.55), SCREEN_SIZE[1] - 56)
    right_rect = pygame.Rect(left_rect.right + 20, 28, SCREEN_SIZE[0] - left_rect.right - 48, SCREEN_SIZE[1] - 56)

    pygame.draw.rect(surf, COLOR_PANEL, left_rect, border_radius=18)
    pygame.draw.rect(surf, COLOR_PANEL_ALT, right_rect, border_radius=18)

    title = font_title.render("VEX Push Back Sim", True, COLOR_TEXT)
    surf.blit(title, (left_rect.x + 24, left_rect.y + 22))
    surf.blit(font_big.render("Team Login", True, COLOR_ACCENT), (left_rect.x + 26, left_rect.y + 86))
    for i, line in enumerate(_wrap_text("Enter your team details or launch a guest run.", 44)):
        surf.blit(font_small.render(line, True, COLOR_TEXT), (left_rect.x + 26, left_rect.y + 124 + i * 20))

    field_w = left_rect.width - 52
    number_rect = pygame.Rect(left_rect.x + 26, left_rect.y + 170, field_w, 54)
    name_rect = pygame.Rect(left_rect.x + 26, left_rect.y + 248, field_w, 54)
    hitboxes = {
        "team_number": number_rect.copy(),
        "team_name": name_rect.copy(),
        "buttons": {},
    }

    _draw_input_box(surf, font_small, "Team Number", team_number, number_rect, active_field == "team_number")
    _draw_input_box(surf, font_small, "Team Name", team_name, name_rect, active_field == "team_name")

    info_lines = [
        "TAB: switch field",
        "ENTER: configure ranked run",
        "G: quick guest run",
        "ESC: ask to quit",
        "",
        "Guest runs are not saved.",
        "Ranked runs use the saved team robot profile.",
    ]

    y = name_rect.bottom + 26
    for line in info_lines:
        surf.blit(font_small.render(line, True, COLOR_TEXT), (left_rect.x + 30, y))
        y += 24

    button_specs = [
        ("ENTER", "Ranked Setup", "Tune robot/controller, then start."),
        ("G", "Guest Run", "Play instantly without saving score."),
        ("ESC", "Quit", "Confirm before closing."),
    ]
    button_y = left_rect.bottom - 196
    button_h = 52
    for key_label, title_label, subtitle in button_specs:
        button_rect = pygame.Rect(left_rect.x + 26, button_y, field_w, button_h)
        button_key = {
            "ENTER": "ranked",
            "G": "guest",
            "ESC": "quit",
        }.get(key_label)
        if button_key is not None:
            hitboxes["buttons"][button_key] = button_rect.copy()
        pygame.draw.rect(surf, (46, 56, 70), button_rect, border_radius=14)
        pygame.draw.rect(surf, COLOR_INPUT_IDLE, button_rect, 2, border_radius=14)
        key_badge = pygame.Rect(button_rect.x + 12, button_rect.y + 10, 78, 32)
        pygame.draw.rect(surf, COLOR_ACCENT, key_badge, border_radius=10)
        pygame.draw.rect(surf, (18, 18, 22), key_badge, 1, border_radius=10)
        surf.blit(font_small.render(key_label, True, (14, 16, 22)), (key_badge.x + 18, key_badge.y + 7))
        surf.blit(font_small.render(title_label, True, COLOR_TEXT), (button_rect.x + 104, button_rect.y + 6))
        surf.blit(font_small.render(subtitle, True, (188, 194, 205)), (button_rect.x + 104, button_rect.y + 26))
        button_y += button_h + 10

    surf.blit(font_big.render("Leaderboard", True, COLOR_TEXT), (right_rect.x + 20, right_rect.y + 20))
    surf.blit(font_small.render("Top team scores", True, COLOR_ACCENT), (right_rect.x + 22, right_rect.y + 58))

    status_rect = None
    if status_message:
        status_rect = pygame.Rect(right_rect.x + 16, right_rect.bottom - 120, right_rect.width - 32, 92)

    controls_h = 186
    controls_bottom = (status_rect.y - 16) if status_rect is not None else (right_rect.bottom - 28)
    controls_rect = pygame.Rect(right_rect.x + 16, controls_bottom - controls_h, right_rect.width - 32, controls_h)

    if leaderboard_entries:
        y = right_rect.y + 98
        leaderboard_bottom_limit = controls_rect.y - 16
        max_visible_entries = max(1, (leaderboard_bottom_limit - y) // 58)
        for idx, entry in enumerate(leaderboard_entries[:max_visible_entries], start=1):
            item_rect = pygame.Rect(right_rect.x + 16, y, right_rect.width - 32, 52)
            pygame.draw.rect(surf, COLOR_PANEL, item_rect, border_radius=12)
            rank_text = font_big.render(f"#{idx}", True, COLOR_ACCENT)
            surf.blit(rank_text, (item_rect.x + 12, item_rect.y + 10))

            team_label = f"{entry.get('team_number', '').strip()}  {entry.get('team_name', '').strip()}".strip()
            team_label = team_label if team_label else "Unknown Team"
            surf.blit(font_small.render(team_label[:28], True, COLOR_TEXT), (item_rect.x + 72, item_rect.y + 10))
            surf.blit(font_small.render(str(entry.get("timestamp", ""))[:16], True, (170, 175, 190)), (item_rect.x + 72, item_rect.y + 28))
            score_surf = font_big.render(str(entry.get("score", 0)), True, COLOR_SUCCESS)
            score_rect = score_surf.get_rect(midright=(item_rect.right - 16, item_rect.y + 26))
            surf.blit(score_surf, score_rect)
            y += 58
    else:
        surf.blit(font_small.render("No saved scores yet.", True, COLOR_TEXT), (right_rect.x + 22, right_rect.y + 102))

    pygame.draw.rect(surf, COLOR_PANEL, controls_rect, border_radius=14)
    pygame.draw.rect(surf, COLOR_INPUT_IDLE, controls_rect, 2, border_radius=14)
    surf.blit(font_big.render("Controls", True, COLOR_TEXT), (controls_rect.x + 16, controls_rect.y + 14))
    control_lines = [
        "Left joystick: forward / back",
        "Right joystick: left / right",
        "X: toggle intake",
        "Arrow down: match loader",
        "R1 or R2: ball dumping",
    ]
    y = controls_rect.y + 54
    for line in control_lines:
        surf.blit(font_small.render(line, True, COLOR_TEXT), (controls_rect.x + 18, y))
        y += 24

    if status_message and status_rect is not None:
        msg_rect = status_rect
        pygame.draw.rect(surf, (40, 58, 48), msg_rect, border_radius=14)
        pygame.draw.rect(surf, COLOR_SUCCESS, msg_rect, 2, border_radius=14)
        max_chars = max(18, (msg_rect.width - 32) // 10)
        max_lines = max(1, (msg_rect.height - 24) // 20)
        wrapped = _wrap_text(status_message, max_chars)
        if len(wrapped) > max_lines:
            wrapped = wrapped[:max_lines]
            if len(wrapped[-1]) > 3:
                wrapped[-1] = wrapped[-1][:-3].rstrip() + "..."
        for i, line in enumerate(wrapped):
            surf.blit(font_small.render(line, True, COLOR_TEXT), (msg_rect.x + 16, msg_rect.y + 12 + i * 20))

    return hitboxes


def draw_robot_config_screen(
    surf,
    font_small,
    font_big,
    font_title,
    profile,
    draft,
    active_field,
    status_message="",
    controller_name="Keyboard",
):
    surf.fill(COLOR_BG)

    left_rect = pygame.Rect(24, 24, int(SCREEN_SIZE[0] * 0.49) - 30, SCREEN_SIZE[1] - 48)
    right_rect = pygame.Rect(left_rect.right + 20, 24, SCREEN_SIZE[0] - left_rect.right - 44, SCREEN_SIZE[1] - 48)

    pygame.draw.rect(surf, COLOR_PANEL, left_rect, border_radius=18)
    pygame.draw.rect(surf, COLOR_PANEL_ALT, right_rect, border_radius=18)

    values = draft.get("values", {})
    hitboxes = {
        "fields": {},
        "buttons": {},
        "toggles": {},
    }

    title = font_title.render("Robot Profile", True, COLOR_TEXT)
    surf.blit(title, (left_rect.x + 24, left_rect.y + 18))
    team_label = f"{profile.team_number}  {profile.team_name}".strip()
    surf.blit(font_big.render(team_label, True, COLOR_ACCENT), (left_rect.x + 26, left_rect.y + 74))

    unit_y = left_rect.y + 120
    surf.blit(font_small.render("Length unit", True, COLOR_TEXT), (left_rect.x + 28, unit_y - 22))
    _draw_segmented_pair(
        surf,
        font_small,
        pygame.Rect(left_rect.x + 26, unit_y, 146, 34),
        "in",
        "cm",
        draft.get("length_unit", "in"),
        hitboxes["toggles"],
        "length_unit",
    )
    surf.blit(font_small.render("Weight unit", True, COLOR_TEXT), (left_rect.x + 206, unit_y - 22))
    _draw_segmented_pair(
        surf,
        font_small,
        pygame.Rect(left_rect.x + 204, unit_y, 146, 34),
        "kg",
        "lb",
        draft.get("mass_unit", "kg"),
        hitboxes["toggles"],
        "mass_unit",
    )

    field_w = int((left_rect.width - 70) / 2)
    field_h = 50
    x1 = left_rect.x + 26
    x2 = x1 + field_w + 18
    y = unit_y + 58
    robot_fields = [
        ("width", "Width", x1, y),
        ("length", "Length", x2, y),
        ("mass", "Weight", x1, y + 72),
        ("drivetrain_rpm", "Drive RPM", x2, y + 72),
        ("intake_rpm", "Intake RPM", x1, y + 144),
        ("wheel_diameter", "Drive wheel", x2, y + 144),
    ]
    for key, label, x, fy in robot_fields:
        rect = pygame.Rect(x, fy, field_w, field_h)
        hitboxes["fields"][key] = rect.copy()
        unit = ""
        if key in {"width", "length", "wheel_diameter"}:
            unit = draft.get("length_unit", "in")
        elif key == "mass":
            unit = draft.get("mass_unit", "kg")
        _draw_input_box(surf, font_small, f"{label} {unit}".strip(), values.get(key, ""), rect, active_field == key)

    outtake_y = y + 232
    surf.blit(font_big.render("Intake / Outtake", True, COLOR_TEXT), (left_rect.x + 26, outtake_y))
    desc_y = outtake_y + 34
    for line in _wrap_text("Same direction unloads from the intake side; opposite unloads from the back.", 50):
        surf.blit(font_small.render(line, True, (190, 196, 208)), (left_rect.x + 28, desc_y))
        desc_y += 19
    _draw_segmented_pair(
        surf,
        font_small,
        pygame.Rect(left_rect.x + 26, desc_y + 8, left_rect.width - 52, 40),
        "front",
        "back",
        draft.get("outtake", "front"),
        hitboxes["toggles"],
        "outtake",
        labels=("Same / front", "Opposite / back"),
    )

    help_lines = [
        "Saved profiles are keyed by team number",
        "and can be changed before any ranked runs.",
    ]
    hy = left_rect.bottom - 104
    for line in help_lines:
        for wrapped in _wrap_text(line, 58):
            surf.blit(font_small.render(wrapped, True, (200, 205, 216)), (left_rect.x + 28, hy))
            hy += 20

    surf.blit(font_title.render("Controller", True, COLOR_TEXT), (right_rect.x + 22, right_rect.y + 18))
    detected_text = f"Detected: {controller_name}"
    if len(detected_text) > 44:
        detected_text = detected_text[:41].rstrip() + "..."
    surf.blit(font_small.render(detected_text, True, COLOR_ACCENT), (right_rect.x + 24, right_rect.y + 70))

    cx1 = right_rect.x + 24
    cx2 = cx1 + field_w + 18
    cy = right_rect.y + 108
    controller_fields = [
        ("forward_axis", "Forward axis", cx1, cy),
        ("turn_axis", "Turn axis", cx2, cy),
        ("deadzone", "Deadzone", cx1, cy + 72),
        ("axis_exponent", "Stick curve", cx2, cy + 72),
        ("intake_binding", "Intake button", cx1, cy + 166),
        ("loader_binding", "Loader button", cx2, cy + 166),
        ("dump_binding", "Outtake button", cx1, cy + 238),
    ]
    for key, label, x, fy in controller_fields:
        rect = pygame.Rect(x, fy, field_w, field_h)
        hitboxes["fields"][key] = rect.copy()
        value = values.get(key, "")
        if key.endswith("_binding") and active_field == key and not value:
            value = "press input"
        _draw_input_box(surf, font_small, label, value, rect, active_field == key)

    invert_y = cy + 144
    _draw_checkbox(
        surf,
        font_small,
        pygame.Rect(cx1, invert_y, field_w, 28),
        "Invert forward",
        bool(draft.get("invert_forward", False)),
        hitboxes["toggles"],
        "invert_forward",
    )
    _draw_checkbox(
        surf,
        font_small,
        pygame.Rect(cx2, invert_y, field_w, 28),
        "Invert turn",
        bool(draft.get("invert_turn", False)),
        hitboxes["toggles"],
        "invert_turn",
    )

    status_rect = pygame.Rect(right_rect.x + 20, right_rect.bottom - 150, right_rect.width - 40, 60)
    if status_message:
        pygame.draw.rect(surf, (40, 58, 48), status_rect, border_radius=12)
        pygame.draw.rect(surf, COLOR_SUCCESS, status_rect, 2, border_radius=12)
        sy = status_rect.y + 10
        for line in _wrap_text(status_message, 50)[:2]:
            surf.blit(font_small.render(line, True, COLOR_TEXT), (status_rect.x + 14, sy))
            sy += 20

    button_y = right_rect.bottom - 70
    button_w = int((right_rect.width - 56) / 3)
    buttons = [
        ("back", "Back"),
        ("save", "Save"),
        ("start", "Start Ranked"),
    ]
    for idx, (key, label) in enumerate(buttons):
        rect = pygame.Rect(right_rect.x + 20 + idx * (button_w + 8), button_y, button_w, 44)
        hitboxes["buttons"][key] = rect.copy()
        accent = COLOR_SUCCESS if key == "start" else COLOR_ACCENT if key == "save" else COLOR_INPUT_IDLE
        _draw_action_button(surf, font_small, rect, label, accent)

    return hitboxes


def draw_quit_confirmation(surf, font_small, font_big):
    overlay = pygame.Surface(SCREEN_SIZE, pygame.SRCALPHA)
    overlay.fill((0, 0, 0, 150))
    surf.blit(overlay, (0, 0))

    dialog = pygame.Rect(0, 0, 430, 190)
    dialog.center = (SCREEN_SIZE[0] // 2, SCREEN_SIZE[1] // 2)
    pygame.draw.rect(surf, COLOR_PANEL_ALT, dialog, border_radius=18)
    pygame.draw.rect(surf, COLOR_WARN, dialog, 2, border_radius=18)

    surf.blit(font_big.render("Exit simulator?", True, COLOR_TEXT), (dialog.x + 28, dialog.y + 26))
    msg_y = dialog.y + 70
    for line in _wrap_text("Press Enter/Y to quit or Esc/N to stay in the menu.", 42):
        surf.blit(font_small.render(line, True, (205, 210, 220)), (dialog.x + 30, msg_y))
        msg_y += 20

    no_rect = pygame.Rect(dialog.x + 34, dialog.bottom - 64, 160, 42)
    yes_rect = pygame.Rect(dialog.right - 194, dialog.bottom - 64, 160, 42)
    _draw_action_button(surf, font_small, no_rect, "Stay", COLOR_ACCENT)
    _draw_action_button(surf, font_small, yes_rect, "Exit", COLOR_WARN)
    return {"no": no_rect, "yes": yes_rect}


def _draw_input_box(surf, font_small, label, value, rect, active):
    border = COLOR_INPUT_ACTIVE if active else COLOR_INPUT_IDLE
    pygame.draw.rect(surf, COLOR_PANEL_ALT, rect, border_radius=12)
    pygame.draw.rect(surf, border, rect, 2, border_radius=12)
    surf.blit(font_small.render(label, True, COLOR_TEXT), (rect.x + 14, rect.y + 6))
    display_value = value if value else ("_" if active else "")
    surf.blit(font_small.render(display_value, True, COLOR_TEXT), (rect.x + 14, rect.y + 28))


def _draw_action_button(surf, font_small, rect, label, accent):
    pygame.draw.rect(surf, (46, 56, 70), rect, border_radius=12)
    pygame.draw.rect(surf, accent, rect, 2, border_radius=12)
    text = font_small.render(label, True, COLOR_TEXT)
    surf.blit(text, text.get_rect(center=rect.center))


def _draw_segmented_pair(surf, font_small, rect, left_value, right_value, selected, hitboxes, prefix, labels=None):
    labels = labels or (left_value, right_value)
    left_rect = pygame.Rect(rect.x, rect.y, rect.width // 2, rect.height)
    right_rect = pygame.Rect(left_rect.right, rect.y, rect.width - left_rect.width, rect.height)
    for value, label, item_rect in ((left_value, labels[0], left_rect), (right_value, labels[1], right_rect)):
        is_selected = selected == value
        color = COLOR_ACCENT if is_selected else (46, 56, 70)
        text_color = (14, 16, 22) if is_selected else COLOR_TEXT
        pygame.draw.rect(surf, color, item_rect, border_radius=10)
        pygame.draw.rect(surf, COLOR_INPUT_IDLE, item_rect, 1, border_radius=10)
        text = font_small.render(str(label), True, text_color)
        surf.blit(text, text.get_rect(center=item_rect.center))
        hitboxes[f"{prefix}:{value}"] = item_rect.copy()


def _draw_checkbox(surf, font_small, rect, label, checked, hitboxes, key):
    box = pygame.Rect(rect.x, rect.y + 3, 22, 22)
    pygame.draw.rect(surf, COLOR_PANEL, box, border_radius=5)
    pygame.draw.rect(surf, COLOR_ACCENT if checked else COLOR_INPUT_IDLE, box, 2, border_radius=5)
    if checked:
        pygame.draw.line(surf, COLOR_SUCCESS, (box.x + 5, box.y + 12), (box.x + 10, box.y + 17), 3)
        pygame.draw.line(surf, COLOR_SUCCESS, (box.x + 10, box.y + 17), (box.x + 18, box.y + 6), 3)
    surf.blit(font_small.render(label, True, COLOR_TEXT), (box.right + 8, rect.y + 5))
    hitboxes[key] = rect.copy()


def _wrap_text(text, max_chars):
    words = text.split()
    if not words:
        return []

    lines = []
    current = words[0]
    for word in words[1:]:
        if len(current) + 1 + len(word) <= max_chars:
            current += " " + word
        else:
            lines.append(current)
            current = word
    lines.append(current)
    return lines
