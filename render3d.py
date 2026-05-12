import pygame

from config import (
    FIELD_SIZE_PX,
    LOADER_CAPACITY,
    LOADER_RADIUS_PX,
    px_to_meters,
)
from physics import bullet


VIEWPORT_RECT = pygame.Rect(0, 0, FIELD_SIZE_PX, FIELD_SIZE_PX)
RENDER_QUALITY_PRESETS = (
    ("fast", 0.30),
    ("balanced", 0.45),
    ("sharp", 0.60),
)
DEFAULT_RENDER_QUALITY_INDEX = 0
MIN_RENDER_SIZE = 200

FIELD_RGBA = (0.24, 0.27, 0.25, 1.0)
FIELD_WALL_RGBA = (0.12, 0.13, 0.15, 1.0)
RED_BLOCK_RGBA = (1.0, 0.18, 0.12, 1.0)
BLUE_BLOCK_RGBA = (0.10, 0.30, 1.0, 1.0)
PARK_RED_RGBA = (0.84, 0.20, 0.18, 1.0)
PARK_BLUE_RGBA = (0.08, 0.48, 0.82, 1.0)
LONG_FLOOR_RGBA = (0.14, 0.48, 0.20, 1.0)
LONG_WALL_RGBA = (0.22, 0.86, 0.36, 1.0)
LONG_SUPPORT_RGBA = (0.16, 0.28, 0.18, 1.0)
MIDDLE_FLOOR_RGBA = (0.13, 0.40, 0.55, 1.0)
MIDDLE_WALL_RGBA = (0.30, 0.86, 1.0, 1.0)
MIDDLE_HIGH_FLOOR_RGBA = (0.24, 0.55, 0.70, 1.0)
MIDDLE_HIGH_WALL_RGBA = (0.64, 0.94, 1.0, 1.0)
ROBOT_BODY_RGBA = (0.70, 0.74, 0.78, 1.0)
ROBOT_BUMPER_RGBA = (0.14, 0.15, 0.18, 1.0)
ROBOT_PADDLE_ACTIVE_RGBA = (1.0, 0.80, 0.22, 1.0)
ROBOT_PADDLE_IDLE_RGBA = (0.48, 0.38, 0.18, 0.42)
ROBOT_ROD_ACTIVE_RGBA = (0.80, 0.82, 0.84, 1.0)
ROBOT_ROD_IDLE_RGBA = (0.36, 0.38, 0.40, 0.42)
LOADER_BASE_RGBA = (1.0, 0.56, 0.10, 0.70)
LOADER_EMPTY_RGBA = (0.16, 0.16, 0.18, 0.45)
DUMP_ZONE_RGBA = (1.0, 0.85, 0.05, 0.32)


def draw_match_3d(
    surf,
    space,
    goals_manager,
    loaders_manager,
    robot,
    *,
    quality_index=DEFAULT_RENDER_QUALITY_INDEX,
):
    if bullet is None or not getattr(space, "connected", False):
        raise RuntimeError("PyBullet renderer is not available.")

    _sync_visual_colors(space, goals_manager, robot)
    _ensure_aux_visuals(space, goals_manager, loaders_manager)
    _sync_loader_queue_visuals(space, loaders_manager)

    render_size = _render_size_for_quality(quality_index)
    frame = _render_camera(space, render_size).convert(surf)
    if render_size == VIEWPORT_RECT.size:
        surf.blit(frame, VIEWPORT_RECT.topleft)
    else:
        pygame.transform.scale(frame, VIEWPORT_RECT.size, surf.subsurface(VIEWPORT_RECT))
    pygame.draw.rect(surf, (8, 10, 12), VIEWPORT_RECT, 2)


def render_quality_name(quality_index):
    return RENDER_QUALITY_PRESETS[_clamp_quality_index(quality_index)][0]


def next_render_quality_index(quality_index):
    return (quality_index + 1) % len(RENDER_QUALITY_PRESETS)


def _clamp_quality_index(quality_index):
    try:
        quality_index = int(quality_index)
    except (TypeError, ValueError):
        quality_index = DEFAULT_RENDER_QUALITY_INDEX
    return max(0, min(len(RENDER_QUALITY_PRESETS) - 1, quality_index))


def _render_size_for_quality(quality_index):
    _, scale = RENDER_QUALITY_PRESETS[_clamp_quality_index(quality_index)]
    width = max(MIN_RENDER_SIZE, int(round(VIEWPORT_RECT.width * scale)))
    height = max(MIN_RENDER_SIZE, int(round(VIEWPORT_RECT.height * scale)))
    return width, height


def _render_camera(space, size):
    width, height = size
    field_m = px_to_meters(FIELD_SIZE_PX)

    # Match the OpenGL camera: lower, less top-down, and from the blue-wall side.
    eye = [field_m * 0.5, -0.90, 2.60]
    target = [field_m * 0.5, field_m * 0.70, 0.0]
    view_matrix = bullet.computeViewMatrix(
        cameraEyePosition=eye,
        cameraTargetPosition=target,
        cameraUpVector=[0.0, 0.0, 1.0],
    )
    projection_matrix = bullet.computeProjectionMatrixFOV(
        fov=84.0,
        aspect=width / max(1, height),
        nearVal=0.02,
        farVal=8.0,
    )

    image = bullet.getCameraImage(
        width,
        height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        lightDirection=[-0.35, -0.45, -1.0],
        lightColor=[1.0, 0.96, 0.88],
        lightDistance=4.0,
        shadow=0,
        renderer=bullet.ER_TINY_RENDERER,
        flags=bullet.ER_NO_SEGMENTATION_MASK,
        physicsClientId=space.client_id,
    )
    rgb = image[2]
    raw = rgb.tobytes() if hasattr(rgb, "tobytes") else bytes(rgb)
    return pygame.image.frombuffer(raw, (width, height), "RGBA").copy()


def _sync_visual_colors(space, goals_manager, robot):
    long_ids = {id(shape) for shape in getattr(goals_manager, "long_shapes", [])}
    middle_ids = {id(shape) for shape in getattr(goals_manager, "middle_shapes", [])}

    for shape in getattr(space, "shapes", []):
        rgba = _rgba_for_shape(shape, long_ids, middle_ids)
        if rgba is not None:
            _set_visual_rgba(space, shape, -1, rgba)

    if robot is not None:
        robot_shape = getattr(robot, "shape", None)
        if robot_shape is not None:
            _set_visual_rgba(space, robot_shape, -1, ROBOT_BODY_RGBA)
            for link_index in getattr(robot_shape, "robot_bumper_link_indices", ()):
                _set_visual_rgba(space, robot_shape, link_index, ROBOT_BUMPER_RGBA)

            paddle_active = bool(getattr(robot, "loader_paddle_active", False))
            for idx, link_index in enumerate(getattr(robot_shape, "loader_paddle_link_indices", ())):
                if idx < 2:
                    rgba = ROBOT_ROD_ACTIVE_RGBA if paddle_active else ROBOT_ROD_IDLE_RGBA
                else:
                    rgba = ROBOT_PADDLE_ACTIVE_RGBA if paddle_active else ROBOT_PADDLE_IDLE_RGBA
                _set_visual_rgba(space, robot_shape, link_index, rgba)


def _rgba_for_shape(shape, long_ids, middle_ids):
    if getattr(shape, "is_block", False):
        return BLUE_BLOCK_RGBA if getattr(shape, "block_color", "red") == "blue" else RED_BLOCK_RGBA

    if getattr(shape, "is_field_floor", False):
        return FIELD_RGBA

    if getattr(shape, "is_field_wall", False):
        return FIELD_WALL_RGBA

    if getattr(shape, "is_parking_zone_edge", False):
        return PARK_BLUE_RGBA if getattr(shape, "parking_zone_color", "") == "blue" else PARK_RED_RGBA

    if id(shape) in long_ids:
        role = getattr(shape, "goal_render_role", "")
        if role == "floor":
            return LONG_FLOOR_RGBA
        if role in {"support", "under_blocker"}:
            return LONG_SUPPORT_RGBA
        return LONG_WALL_RGBA

    if id(shape) in middle_ids:
        role = getattr(shape, "goal_render_role", "")
        is_high = getattr(shape, "goal_height_role", "") == "high"
        if role == "floor":
            return MIDDLE_HIGH_FLOOR_RGBA if is_high else MIDDLE_FLOOR_RGBA
        return MIDDLE_HIGH_WALL_RGBA if is_high else MIDDLE_WALL_RGBA

    return None


def _set_visual_rgba(space, shape, link_index, rgba):
    key = "_render3d_rgba" if link_index == -1 else f"_render3d_rgba_{link_index}"
    rounded = tuple(round(float(v), 4) for v in rgba)
    if getattr(shape, key, None) == rounded:
        return
    bullet.changeVisualShape(
        shape.body.body_id,
        link_index,
        rgbaColor=list(rounded),
        physicsClientId=space.client_id,
    )
    setattr(shape, key, rounded)


def _ensure_aux_visuals(space, goals_manager, loaders_manager):
    dump_sig = tuple(
        (zone.rect.x, zone.rect.y, zone.rect.width, zone.rect.height)
        for zone in getattr(goals_manager, "dump_zones", [])
    )
    loader_sig = tuple(
        (loader.name, round(loader.pos.x, 2), round(loader.pos.y, 2))
        for loader in getattr(loaders_manager, "loaders", [])
    )
    signature = (dump_sig, loader_sig)

    aux = getattr(space, "_render3d_aux_visuals", None)
    if aux is not None and aux.get("signature") == signature:
        return

    if aux is not None:
        for body_id in aux.get("body_ids", []):
            bullet.removeBody(body_id, physicsClientId=space.client_id)

    body_ids = []
    loader_slot_body_ids = []

    for zone in getattr(goals_manager, "dump_zones", []):
        rect = zone.rect
        half_extents = [
            px_to_meters(rect.width * 0.5),
            px_to_meters(rect.height * 0.5),
            0.004,
        ]
        visual_shape = bullet.createVisualShape(
            bullet.GEOM_BOX,
            halfExtents=half_extents,
            rgbaColor=list(DUMP_ZONE_RGBA),
            specularColor=[0.05, 0.05, 0.02],
            physicsClientId=space.client_id,
        )
        body_id = bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=visual_shape,
            basePosition=[
                px_to_meters(rect.centerx),
                px_to_meters(rect.centery),
                0.008,
            ],
            physicsClientId=space.client_id,
        )
        body_ids.append(body_id)

    loader_base_shape = bullet.createVisualShape(
        bullet.GEOM_CYLINDER,
        radius=px_to_meters(LOADER_RADIUS_PX),
        length=0.050,
        rgbaColor=list(LOADER_BASE_RGBA),
        specularColor=[0.25, 0.20, 0.10],
        physicsClientId=space.client_id,
    )
    loader_slot_shape = bullet.createVisualShape(
        bullet.GEOM_SPHERE,
        radius=max(0.018, px_to_meters(LOADER_RADIUS_PX * 0.34)),
        rgbaColor=list(LOADER_EMPTY_RGBA),
        specularColor=[0.22, 0.22, 0.22],
        physicsClientId=space.client_id,
    )

    for loader in getattr(loaders_manager, "loaders", []):
        base_id = bullet.createMultiBody(
            baseMass=0.0,
            baseCollisionShapeIndex=-1,
            baseVisualShapeIndex=loader_base_shape,
            basePosition=[px_to_meters(loader.pos.x), px_to_meters(loader.pos.y), 0.030],
            physicsClientId=space.client_id,
        )
        body_ids.append(base_id)

        slot_ids = []
        inside_sign = 1.0 if loader.pos.y < (FIELD_SIZE_PX * 0.5) else -1.0
        row_y = loader.pos.y + inside_sign * (LOADER_RADIUS_PX + 16.0)
        slot_gap = LOADER_RADIUS_PX * 0.58
        start_x = loader.pos.x - ((LOADER_CAPACITY - 1) * slot_gap * 0.5)
        for i in range(LOADER_CAPACITY):
            slot_id = bullet.createMultiBody(
                baseMass=0.0,
                baseCollisionShapeIndex=-1,
                baseVisualShapeIndex=loader_slot_shape,
                basePosition=[
                    px_to_meters(start_x + (i * slot_gap)),
                    px_to_meters(row_y),
                    0.075,
                ],
                physicsClientId=space.client_id,
            )
            body_ids.append(slot_id)
            slot_ids.append(slot_id)
        loader_slot_body_ids.append(slot_ids)

    space._render3d_aux_visuals = {
        "signature": signature,
        "body_ids": body_ids,
        "loader_slots": loader_slot_body_ids,
        "loader_slot_rgba": {},
    }


def _sync_loader_queue_visuals(space, loaders_manager):
    aux = getattr(space, "_render3d_aux_visuals", None)
    if aux is None:
        return

    current_rgba = aux.setdefault("loader_slot_rgba", {})
    loaders = list(getattr(loaders_manager, "loaders", []))
    for loader_index, loader in enumerate(loaders):
        if loader_index >= len(aux.get("loader_slots", [])):
            break
        slot_ids = aux["loader_slots"][loader_index]
        for slot_index, body_id in enumerate(slot_ids):
            if slot_index < len(loader.queue):
                color = loader.queue[slot_index]
                rgba = BLUE_BLOCK_RGBA if color == "blue" else RED_BLOCK_RGBA
            else:
                rgba = LOADER_EMPTY_RGBA

            rounded = tuple(round(float(v), 4) for v in rgba)
            key = (loader_index, slot_index)
            if current_rgba.get(key) == rounded:
                continue
            bullet.changeVisualShape(
                body_id,
                -1,
                rgbaColor=list(rounded),
                physicsClientId=space.client_id,
            )
            current_rgba[key] = rounded
