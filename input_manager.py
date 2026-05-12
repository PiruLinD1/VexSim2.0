import math
from dataclasses import dataclass

import pygame

from config import (
    GAMEPAD_DEADZONE,
    GAMEPAD_AXIS_EXPONENT,
    VEX_CONTROLLER_NAME_HINTS,
    VEX_FORWARD_AXIS,
    VEX_TURN_AXIS,
    VEX_INVERT_FORWARD,
    VEX_INVERT_TURN,
    VEX_BUTTON_INTAKE_TOGGLE,
    VEX_BUTTON_DUMP_HOLD,
    VEX_LOADER_HAT_INDEX,
    VEX_LOADER_HAT_VECTOR,
    GENERIC_FORWARD_AXIS,
    GENERIC_TURN_AXIS,
    GENERIC_INVERT_FORWARD,
    GENERIC_INVERT_TURN,
    GENERIC_BUTTON_INTAKE_TOGGLE,
    GENERIC_BUTTON_DUMP_HOLD,
    GENERIC_LOADER_HAT_INDEX,
    GENERIC_LOADER_HAT_VECTOR,
)
from team_config import ControllerSettings as TeamControllerSettings, binding_pressed


@dataclass(frozen=True)
class ControllerProfile:
    name: str
    name_hints: tuple[str, ...]
    forward_axis: int
    turn_axis: int
    invert_forward: bool
    invert_turn: bool
    intake_toggle_buttons: tuple[int, ...]
    dump_hold_buttons: tuple[int, ...]
    loader_hat_index: int
    loader_hat_vector: tuple[int, int]

    def matches_name(self, device_name: str) -> bool:
        lower_name = device_name.lower()
        return any(hint in lower_name for hint in self.name_hints)


@dataclass
class ControlState:
    forward: float = 0.0
    turn: float = 0.0
    intake_enabled: bool = False
    loader_active: bool = False
    dump_toggle: bool = False
    dump_hold: bool = False
    controller_connected: bool = False
    controller_name: str = "Keyboard"
    controller_profile: str = "keyboard"
    deadzone: float = GAMEPAD_DEADZONE
    axis_exponent: float = GAMEPAD_AXIS_EXPONENT
    pressed_buttons: tuple[int, ...] = ()
    hat_state: tuple[tuple[int, int], ...] = ()


VEX_USB_PROFILE = ControllerProfile(
    name="vex_usb",
    name_hints=VEX_CONTROLLER_NAME_HINTS,
    forward_axis=VEX_FORWARD_AXIS,
    turn_axis=VEX_TURN_AXIS,
    invert_forward=VEX_INVERT_FORWARD,
    invert_turn=VEX_INVERT_TURN,
    intake_toggle_buttons=VEX_BUTTON_INTAKE_TOGGLE,
    dump_hold_buttons=VEX_BUTTON_DUMP_HOLD,
    loader_hat_index=VEX_LOADER_HAT_INDEX,
    loader_hat_vector=VEX_LOADER_HAT_VECTOR,
)

GENERIC_GAMEPAD_PROFILE = ControllerProfile(
    name="generic_gamepad",
    name_hints=(),
    forward_axis=GENERIC_FORWARD_AXIS,
    turn_axis=GENERIC_TURN_AXIS,
    invert_forward=GENERIC_INVERT_FORWARD,
    invert_turn=GENERIC_INVERT_TURN,
    intake_toggle_buttons=GENERIC_BUTTON_INTAKE_TOGGLE,
    dump_hold_buttons=GENERIC_BUTTON_DUMP_HOLD,
    loader_hat_index=GENERIC_LOADER_HAT_INDEX,
    loader_hat_vector=GENERIC_LOADER_HAT_VECTOR,
)


class InputManager:
    def __init__(self):
        pygame.joystick.init()

        self.active_joystick = None
        self.active_profile = GENERIC_GAMEPAD_PROFILE
        self.prev_dump_pressed = False
        self.prev_intake_toggle_pressed = False
        self.prev_loader_toggle_pressed = False
        self.controller_intake_toggle = False
        self.controller_loader_toggle = False
        self.controller_settings: TeamControllerSettings | None = None
        self.last_state = ControlState()

        self.refresh_devices()

    def set_controller_settings(self, settings: TeamControllerSettings | None):
        self.controller_settings = settings
        self.prev_dump_pressed = False
        self.prev_intake_toggle_pressed = False
        self.prev_loader_toggle_pressed = False
        self.controller_intake_toggle = False
        self.controller_loader_toggle = False

    def handle_event(self, event):
        if event.type in (pygame.JOYDEVICEADDED, pygame.JOYDEVICEREMOVED):
            self.refresh_devices()

    def refresh_devices(self):
        self.prev_intake_toggle_pressed = False
        self.prev_loader_toggle_pressed = False
        self.controller_intake_toggle = False
        self.controller_loader_toggle = False

        joysticks = []
        for idx in range(pygame.joystick.get_count()):
            joystick = pygame.joystick.Joystick(idx)
            joystick.init()
            joysticks.append(joystick)

        if not joysticks:
            self.active_joystick = None
            self.active_profile = GENERIC_GAMEPAD_PROFILE
            return

        best = None
        for joystick in joysticks:
            if VEX_USB_PROFILE.matches_name(joystick.get_name()):
                best = (joystick, VEX_USB_PROFILE)
                break

        if best is None:
            best = (joysticks[0], GENERIC_GAMEPAD_PROFILE)

        self.active_joystick, self.active_profile = best

    def poll(self, keys) -> ControlState:
        keyboard_forward = 0.0
        keyboard_turn = 0.0
        if keys[pygame.K_w] or keys[pygame.K_UP]:
            keyboard_forward += 1.0
        if keys[pygame.K_s] or keys[pygame.K_DOWN]:
            keyboard_forward -= 1.0
        if keys[pygame.K_a] or keys[pygame.K_LEFT]:
            keyboard_turn -= 1.0
        if keys[pygame.K_d] or keys[pygame.K_RIGHT]:
            keyboard_turn += 1.0

        keyboard_intake = keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]
        keyboard_loader = keys[pygame.K_e]
        keyboard_dump = keys[pygame.K_SPACE]

        pad_forward = 0.0
        pad_turn = 0.0
        pad_intake_toggle_pressed = False
        pad_loader_toggle_pressed = False
        pad_dump_hold = False
        pad_buttons = ()
        hat_state = ()
        controller_name = "Keyboard"
        controller_profile = "keyboard"
        controller_connected = False

        if self.active_joystick is not None:
            controller_connected = True
            controller_name = self.active_joystick.get_name()
            settings = self.controller_settings
            controller_profile = "custom_team" if settings is not None else self.active_profile.name

            forward_axis = settings.forward_axis if settings is not None else self.active_profile.forward_axis
            turn_axis = settings.turn_axis if settings is not None else self.active_profile.turn_axis
            invert_forward = settings.invert_forward if settings is not None else self.active_profile.invert_forward
            invert_turn = settings.invert_turn if settings is not None else self.active_profile.invert_turn
            deadzone = settings.deadzone if settings is not None else GAMEPAD_DEADZONE
            axis_exponent = settings.axis_exponent if settings is not None else GAMEPAD_AXIS_EXPONENT

            pad_forward = self._read_axis(forward_axis, deadzone, axis_exponent)
            if invert_forward:
                pad_forward *= -1.0

            pad_turn = self._read_axis(turn_axis, deadzone, axis_exponent)
            if invert_turn:
                pad_turn *= -1.0

            if settings is not None:
                pad_intake_toggle_pressed = binding_pressed(
                    settings.intake_binding,
                    self._button_down,
                    self._hat_matches,
                )
                pad_loader_toggle_pressed = binding_pressed(
                    settings.loader_binding,
                    self._button_down,
                    self._hat_matches,
                )
                pad_dump_hold = binding_pressed(
                    settings.dump_binding,
                    self._button_down,
                    self._hat_matches,
                )
            else:
                pad_intake_toggle_pressed = any(
                    self._button_down(i) for i in self.active_profile.intake_toggle_buttons
                )
                pad_loader_toggle_pressed = self._hat_matches(
                    self.active_profile.loader_hat_index,
                    self.active_profile.loader_hat_vector,
                )
                pad_dump_hold = any(
                    self._button_down(i) for i in self.active_profile.dump_hold_buttons
                )

            pad_buttons = tuple(
                i for i in range(self.active_joystick.get_numbuttons()) if self._button_down(i)
            )
            hat_state = tuple(
                self.active_joystick.get_hat(i) for i in range(self.active_joystick.get_numhats())
            )

        if pad_intake_toggle_pressed and not self.prev_intake_toggle_pressed:
            self.controller_intake_toggle = not self.controller_intake_toggle
        self.prev_intake_toggle_pressed = pad_intake_toggle_pressed

        if pad_loader_toggle_pressed and not self.prev_loader_toggle_pressed:
            self.controller_loader_toggle = not self.controller_loader_toggle
        self.prev_loader_toggle_pressed = pad_loader_toggle_pressed

        forward = keyboard_forward if abs(keyboard_forward) > 1e-4 else pad_forward
        turn = keyboard_turn if abs(keyboard_turn) > 1e-4 else pad_turn
        intake_enabled = bool(keyboard_intake or self.controller_intake_toggle)
        loader_active = bool(keyboard_loader or self.controller_loader_toggle)

        keyboard_dump_toggle = bool(keyboard_dump)
        dump_toggle = keyboard_dump_toggle and not self.prev_dump_pressed
        self.prev_dump_pressed = keyboard_dump_toggle

        state = ControlState(
            forward=self._clamp(forward, -1.0, 1.0),
            turn=self._clamp(turn, -1.0, 1.0),
            intake_enabled=intake_enabled,
            loader_active=loader_active,
            dump_toggle=dump_toggle,
            dump_hold=pad_dump_hold,
            controller_connected=controller_connected,
            controller_name=controller_name,
            controller_profile=controller_profile,
            deadzone=self.controller_settings.deadzone if self.controller_settings is not None else GAMEPAD_DEADZONE,
            axis_exponent=self.controller_settings.axis_exponent if self.controller_settings is not None else GAMEPAD_AXIS_EXPONENT,
            pressed_buttons=pad_buttons,
            hat_state=hat_state,
        )
        self.last_state = state
        return state

    def _read_axis(self, axis_index: int, deadzone: float, axis_exponent: float) -> float:
        if self.active_joystick is None:
            return 0.0
        if axis_index < 0 or axis_index >= self.active_joystick.get_numaxes():
            return 0.0

        value = float(self.active_joystick.get_axis(axis_index))
        return self._shape_axis(value, deadzone, axis_exponent)

    def _button_down(self, button_index: int) -> bool:
        if self.active_joystick is None:
            return False
        if button_index < 0 or button_index >= self.active_joystick.get_numbuttons():
            return False
        return bool(self.active_joystick.get_button(button_index))

    def _hat_matches(self, hat_index: int, expected: tuple[int, int]) -> bool:
        if self.active_joystick is None:
            return False
        if hat_index < 0 or hat_index >= self.active_joystick.get_numhats():
            return False
        return self.active_joystick.get_hat(hat_index) == expected

    def _shape_axis(self, value: float, deadzone: float, axis_exponent: float) -> float:
        mag = abs(value)
        deadzone = self._clamp(float(deadzone), 0.0, 0.95)
        axis_exponent = max(0.05, float(axis_exponent))
        if mag <= deadzone:
            return 0.0

        mag = (mag - deadzone) / max(1e-6, 1.0 - deadzone)
        shaped = mag ** axis_exponent
        return math.copysign(self._clamp(shaped, 0.0, 1.0), value)

    @staticmethod
    def _clamp(x: float, a: float, b: float) -> float:
        return a if x < a else b if x > b else x
