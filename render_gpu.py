import math
import os

import pygame

from config import (
    FIELD_SIZE_PX,
    LOADER_CAPACITY,
    LOADER_RADIUS_PX,
    LOADER_PADDLE_LEN_PX,
    LOADER_PADDLE_T_PX,
    LOADER_PADDLE_W_PX,
    PARK_ZONE_OUTER_SIZE_IN,
    inches_to_px,
    px_to_meters,
)
from physics import TRIBALL_MESH


try:
    from OpenGL.GL import *  # noqa: F401,F403
    from OpenGL.GLU import *  # noqa: F401,F403
except Exception as exc:  # pragma: no cover - depends on local GPU deps
    GPU_AVAILABLE = False
    GPU_IMPORT_ERROR = exc
else:
    GPU_AVAILABLE = True
    GPU_IMPORT_ERROR = None


FIELD_RGBA = (0.20, 0.24, 0.22, 1.0)
FIELD_ALT_RGBA = (0.24, 0.28, 0.25, 1.0)
FIELD_LINE_RGBA = (0.62, 0.68, 0.62, 0.34)
FIELD_WALL_RGBA = (0.10, 0.11, 0.13, 1.0)
RED_BLOCK_RGBA = (1.0, 0.12, 0.08, 1.0)
BLUE_BLOCK_RGBA = (0.08, 0.30, 1.0, 1.0)
PARK_RED_RGBA = (0.88, 0.18, 0.14, 1.0)
PARK_BLUE_RGBA = (0.06, 0.44, 0.82, 1.0)
LONG_FLOOR_RGBA = (0.10, 0.42, 0.18, 1.0)
LONG_WALL_RGBA = (0.18, 0.86, 0.34, 1.0)
LONG_SUPPORT_RGBA = (0.11, 0.22, 0.14, 1.0)
MIDDLE_FLOOR_RGBA = (0.12, 0.34, 0.48, 1.0)
MIDDLE_WALL_RGBA = (0.25, 0.78, 0.96, 1.0)
MIDDLE_HIGH_FLOOR_RGBA = (0.22, 0.50, 0.66, 1.0)
MIDDLE_HIGH_WALL_RGBA = (0.62, 0.92, 1.0, 1.0)
ROBOT_BODY_RGBA = (0.72, 0.76, 0.78, 1.0)
ROBOT_TOP_RGBA = (0.90, 0.94, 0.96, 1.0)
ROBOT_BUMPER_RGBA = (0.08, 0.09, 0.11, 1.0)
ROBOT_WHEEL_RGBA = (0.02, 0.02, 0.025, 1.0)
ROBOT_PADDLE_ACTIVE_RGBA = (1.0, 0.78, 0.18, 1.0)
ROBOT_PADDLE_IDLE_RGBA = (0.44, 0.34, 0.16, 0.65)
ROBOT_ROD_ACTIVE_RGBA = (0.86, 0.88, 0.90, 1.0)
ROBOT_ROD_IDLE_RGBA = (0.38, 0.40, 0.42, 0.72)
LOADER_BASE_RGBA = (1.0, 0.54, 0.10, 0.92)
LOADER_EMPTY_RGBA = (0.08, 0.08, 0.10, 0.50)
LOADER_TUBE_RGBA = (0.72, 0.92, 1.0, 0.28)
LOADER_TUBE_EDGE_RGBA = (0.86, 0.96, 1.0, 0.62)
LOADER_MOUTH_RGBA = (1.0, 0.58, 0.12, 0.96)
DUMP_ZONE_RGBA = (1.0, 0.85, 0.05, 0.26)
SHADOW_RGBA = (0.0, 0.0, 0.0, 0.22)

LOADER_TUBE_HEIGHT_M = 0.52
LOADER_TUBE_BALL_BOTTOM_M = 0.070
LOADER_TUBE_BALL_STEP_M = 0.075
LOADER_TUBE_GAP_HEIGHT_M = 0.165
LOADER_TUBE_GAP_RAD = math.radians(72.0)

TRIBALL_RINGS = (
    (-1.00, 0.72),
    (-0.52, 0.94),
    (0.00, 1.00),
    (0.52, 0.94),
    (1.00, 0.72),
)
TRIBALL_SIDES = 18
TRIBALL_FLAT_RATIO = 41.0 / 49.0


class GpuDisplay:
    def __init__(self, size):
        if not GPU_AVAILABLE:
            raise RuntimeError(f"PyOpenGL is not installed: {GPU_IMPORT_ERROR}")

        self.size = tuple(size)
        self._textures = {}
        self._static_scene_list = None
        self._static_scene_signature = None
        self._loader_tube_list = None
        self._loader_tube_signature = None
        self._triball_list = None
        self._init_display()
        self._configure_gl()

    def _init_display(self):
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MAJOR_VERSION, 2)
        pygame.display.gl_set_attribute(pygame.GL_CONTEXT_MINOR_VERSION, 1)
        pygame.display.gl_set_attribute(pygame.GL_DOUBLEBUFFER, 1)
        pygame.display.gl_set_attribute(pygame.GL_DEPTH_SIZE, 24)

        last_error = None
        for samples in (4, 0):
            try:
                pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLEBUFFERS, 1 if samples else 0)
                pygame.display.gl_set_attribute(pygame.GL_MULTISAMPLESAMPLES, samples)
                flags = pygame.OPENGL | pygame.DOUBLEBUF
                if os.environ.get("VEXSIM_GPU_FULLSCREEN") == "1":
                    flags |= pygame.FULLSCREEN
                pygame.display.set_mode(self.size, flags)
                self.msaa_samples = samples
                return
            except pygame.error as exc:
                last_error = exc

        raise RuntimeError(f"Could not create an OpenGL display: {last_error}")

    def _configure_gl(self):
        glClearColor(0.035, 0.040, 0.048, 1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glDisable(GL_CULL_FACE)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)
        glHint(GL_LINE_SMOOTH_HINT, GL_NICEST)
        if getattr(self, "msaa_samples", 0):
            glEnable(GL_MULTISAMPLE)

    def present_canvas(self, surface):
        width, height = self.size
        glViewport(0, 0, width, height)
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._setup_2d()
        self._draw_surface(surface, pygame.Rect(0, 0, width, height), "full")
        pygame.display.flip()

    def present_overlay(self, surface):
        glViewport(0, 0, self.size[0], self.size[1])
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        self._setup_2d()
        self._draw_surface(surface, pygame.Rect(0, 0, self.size[0], self.size[1]), "overlay_full")
        glDisable(GL_BLEND)
        pygame.display.flip()

    def present_overlay_regions(self, surface, rects):
        glViewport(0, 0, self.size[0], self.size[1])
        glDisable(GL_DEPTH_TEST)
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        self._setup_2d()
        for rect in rects:
            rect = pygame.Rect(rect)
            if rect.width <= 0 or rect.height <= 0:
                continue
            self._draw_surface(
                surface.subsurface(rect),
                rect,
                ("overlay_region", rect.x, rect.y, rect.width, rect.height),
            )
        glDisable(GL_BLEND)
        pygame.display.flip()

    def draw_match(self, space, goals_manager, loaders_manager, robot, *, quality_index=0):
        del quality_index
        glViewport(0, 0, FIELD_SIZE_PX, FIELD_SIZE_PX)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self._setup_camera()
        self._setup_lighting()

        self._draw_static_scene(space, goals_manager, loaders_manager, robot)
        self._draw_dynamic_shadows(space, robot)
        self._draw_blocks(space)
        self._draw_loader_queue(loaders_manager)
        self._draw_robot(robot)
        self._draw_loader_tubes(loaders_manager)

    def _draw_static_scene(self, space, goals_manager, loaders_manager, robot):
        signature = self._static_signature(space, goals_manager, loaders_manager, robot)
        if self._static_scene_list is None or self._static_scene_signature != signature:
            if self._static_scene_list is not None:
                glDeleteLists(self._static_scene_list, 1)
            self._static_scene_list = glGenLists(1)
            self._static_scene_signature = signature
            glNewList(self._static_scene_list, GL_COMPILE)
            self._draw_floor(goals_manager)
            self._draw_static_shapes(space, goals_manager, robot)
            self._draw_loader_bases(loaders_manager)
            glEndList()
        glCallList(self._static_scene_list)

    def _static_signature(self, space, goals_manager, loaders_manager, robot):
        robot_shape = getattr(robot, "shape", None)
        static_ids = tuple(
            shape.body.body_id
            for shape in getattr(space, "shapes", [])
            if shape is not robot_shape and not getattr(shape, "is_block", False)
        )
        dump_sig = tuple(
            (zone.rect.x, zone.rect.y, zone.rect.width, zone.rect.height)
            for zone in getattr(goals_manager, "dump_zones", [])
        )
        loader_sig = tuple(
            (round(loader.pos.x, 2), round(loader.pos.y, 2))
            for loader in getattr(loaders_manager, "loaders", [])
        )
        return (id(space), static_ids, dump_sig, loader_sig, id(robot_shape))

    def _setup_camera(self):
        field_m = px_to_meters(FIELD_SIZE_PX)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(84.0, 1.0, 0.02, 8.0)

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        # View from the blue-wall side so screen-right matches world +X.
        # The old red-wall camera mirrored left/right controls visually.
        gluLookAt(
            field_m * 0.5,
            -0.90,
            2.20,
            field_m * 0.5,
            field_m * 0.70,
            0.0,
            0.0,
            0.0,
            1.0,
        )

    def _setup_lighting(self):
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, [0.30, 0.33, 0.36, 1.0])
        glLightfv(GL_LIGHT0, GL_POSITION, [-0.35, -0.46, 0.86, 0.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 0.96, 0.88, 1.0])
        glLightfv(GL_LIGHT0, GL_SPECULAR, [0.36, 0.34, 0.30, 1.0])
        glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, [0.22, 0.24, 0.25, 1.0])
        glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 32.0)

    def _draw_floor(self, goals_manager):
        field_m = px_to_meters(FIELD_SIZE_PX)
        tile_m = px_to_meters(inches_to_px(24.0))

        glDisable(GL_LIGHTING)
        _set_color(FIELD_RGBA)
        _quad_xy(0.0, 0.0, field_m, field_m, 0.0)

        # Subtle foam-tile checkerboard.
        for ix in range(6):
            for iy in range(6):
                if (ix + iy) % 2 == 0:
                    continue
                _set_color(FIELD_ALT_RGBA)
                x0 = ix * tile_m
                y0 = iy * tile_m
                _quad_xy(x0, y0, min(x0 + tile_m, field_m), min(y0 + tile_m, field_m), 0.001)

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        _draw_parking_floor_marks()
        for zone in getattr(goals_manager, "dump_zones", []):
            _draw_rect_on_floor(zone.rect, DUMP_ZONE_RGBA, z=0.006)
        glDisable(GL_BLEND)

        _set_color(FIELD_LINE_RGBA)
        glLineWidth(1.0)
        glBegin(GL_LINES)
        for i in range(7):
            p = i * tile_m
            glVertex3f(p, 0.0, 0.004)
            glVertex3f(p, field_m, 0.004)
            glVertex3f(0.0, p, 0.004)
            glVertex3f(field_m, p, 0.004)
        glEnd()
        glEnable(GL_LIGHTING)

    def _draw_static_shapes(self, space, goals_manager, robot):
        long_ids = {id(shape) for shape in getattr(goals_manager, "long_shapes", [])}
        middle_ids = {id(shape) for shape in getattr(goals_manager, "middle_shapes", [])}
        robot_shape = getattr(robot, "shape", None)

        for shape in getattr(space, "shapes", []):
            if shape is robot_shape or getattr(shape, "is_block", False):
                continue
            if getattr(shape, "is_field_floor", False) or getattr(shape, "is_goal_mouth_gate", False):
                continue
            if getattr(shape, "render_rgba", True) is None:
                continue

            rgba = _rgba_for_shape(shape, long_ids, middle_ids)
            if rgba is None:
                rgba = getattr(shape, "render_rgba", (0.55, 0.58, 0.60, 1.0))
            self._draw_shape_box(shape, rgba)

    def _draw_shape_box(self, shape, rgba):
        size_px = getattr(shape, "render_size_px", None)
        if size_px is None:
            size_px = _size_from_vertices(shape)
        width_m = px_to_meters(size_px[0])
        depth_m = px_to_meters(size_px[1])
        height_m = float(getattr(shape, "render_height_m", 0.05))

        glPushMatrix()
        _apply_body_transform(shape.body)
        _draw_box(width_m, depth_m, height_m, rgba)
        glPopMatrix()

    def _draw_loader_bases(self, loaders_manager):
        for loader in getattr(loaders_manager, "loaders", []):
            x = px_to_meters(loader.pos.x)
            y = px_to_meters(loader.pos.y)
            radius = px_to_meters(LOADER_RADIUS_PX)
            inward_angle = _loader_inward_angle(loader)

            glPushMatrix()
            glTranslatef(x, y, 0.0)
            _draw_flat_ring(
                radius * 1.12,
                radius * 0.84,
                0.012,
                LOADER_MOUTH_RGBA,
                gap_angle=inward_angle,
                gap_width=LOADER_TUBE_GAP_RAD,
            )
            glRotatef(math.degrees(inward_angle - (math.pi * 0.5)), 0.0, 0.0, 1.0)
            mouth_len = radius * 1.10
            rail_w = radius * 0.18
            rail_h = 0.045
            for sx in (-1.0, 1.0):
                glPushMatrix()
                glTranslatef(sx * radius * 0.48, radius + (mouth_len * 0.5), rail_h * 0.5)
                _draw_box(rail_w, mouth_len, rail_h, LOADER_MOUTH_RGBA)
                glPopMatrix()
            glPushMatrix()
            glTranslatef(0.0, radius + mouth_len, rail_h * 0.5)
            _draw_box(radius * 1.14, rail_w, rail_h, LOADER_MOUTH_RGBA)
            glPopMatrix()
            glPopMatrix()

    def _draw_loader_queue(self, loaders_manager):
        for loader in getattr(loaders_manager, "loaders", []):
            for slot_index, color_name in enumerate(list(loader.queue)[:LOADER_CAPACITY]):
                color = BLUE_BLOCK_RGBA if color_name == "blue" else RED_BLOCK_RGBA
                glPushMatrix()
                glTranslatef(
                    px_to_meters(loader.pos.x),
                    px_to_meters(loader.pos.y),
                    LOADER_TUBE_BALL_BOTTOM_M + (slot_index * LOADER_TUBE_BALL_STEP_M),
                )
                self._draw_triball(color)
                glPopMatrix()

    def _draw_loader_tubes(self, loaders_manager):
        signature = tuple(
            (round(loader.pos.x, 2), round(loader.pos.y, 2))
            for loader in getattr(loaders_manager, "loaders", [])
        )
        if self._loader_tube_list is None or self._loader_tube_signature != signature:
            if self._loader_tube_list is not None:
                glDeleteLists(self._loader_tube_list, 1)
            self._loader_tube_list = glGenLists(1)
            self._loader_tube_signature = signature
            glNewList(self._loader_tube_list, GL_COMPILE)
            self._draw_loader_tube_geometry(loaders_manager)
            glEndList()

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthMask(GL_FALSE)
        glCallList(self._loader_tube_list)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)

    def _draw_loader_tube_geometry(self, loaders_manager):
        for loader in getattr(loaders_manager, "loaders", []):
            radius = px_to_meters(LOADER_RADIUS_PX)
            inward_angle = _loader_inward_angle(loader)
            glPushMatrix()
            glTranslatef(px_to_meters(loader.pos.x), px_to_meters(loader.pos.y), 0.0)
            _draw_cylinder_wall_with_lower_gap(
                radius,
                LOADER_TUBE_HEIGHT_M,
                LOADER_TUBE_RGBA,
                gap_angle=inward_angle,
                gap_width=LOADER_TUBE_GAP_RAD,
                gap_height=LOADER_TUBE_GAP_HEIGHT_M,
                segments=56,
            )
            _draw_flat_ring(
                radius * 1.04,
                radius * 0.94,
                LOADER_TUBE_HEIGHT_M,
                LOADER_TUBE_EDGE_RGBA,
            )
            _draw_flat_ring(
                radius * 1.04,
                radius * 0.94,
                0.030,
                LOADER_TUBE_EDGE_RGBA,
                gap_angle=inward_angle,
                gap_width=LOADER_TUBE_GAP_RAD,
            )
            glPopMatrix()

    def _draw_dynamic_shadows(self, space, robot):
        glDisable(GL_LIGHTING)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthMask(GL_FALSE)

        for shape in getattr(space, "block_shapes", []):
            pos = shape.body.position
            radius = px_to_meters(LOADER_RADIUS_PX * 0.60)
            _draw_shadow_disc(px_to_meters(pos.x), px_to_meters(pos.y), radius)

        if robot is not None:
            pos = robot.body.position
            sx = px_to_meters(robot.width_px * 0.56)
            sy = px_to_meters(robot.length_px * 0.56)
            glPushMatrix()
            glTranslatef(px_to_meters(pos.x), px_to_meters(pos.y), 0.007)
            glRotatef(math.degrees(robot.body.angle), 0.0, 0.0, 1.0)
            _set_color((0.0, 0.0, 0.0, 0.18))
            _quad_xy(-sx, -sy, sx, sy, 0.0)
            glPopMatrix()

        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)
        glEnable(GL_LIGHTING)

    def _draw_blocks(self, space):
        for shape in getattr(space, "block_shapes", []):
            color = BLUE_BLOCK_RGBA if getattr(shape, "block_color", "red") == "blue" else RED_BLOCK_RGBA
            glPushMatrix()
            _apply_body_transform(shape.body)
            self._draw_triball(color)
            glPopMatrix()

    def _draw_robot(self, robot):
        if robot is None:
            return

        width_m = px_to_meters(robot.width_px)
        depth_m = px_to_meters(robot.length_px)
        height_m = float(getattr(robot.shape, "render_height_m", 0.22))

        glPushMatrix()
        _apply_body_transform(robot.body)
        _draw_box(width_m, depth_m, height_m, ROBOT_BODY_RGBA)

        glPushMatrix()
        glTranslatef(0.0, 0.0, height_m * 0.53)
        _draw_box(width_m * 0.74, depth_m * 0.64, height_m * 0.06, ROBOT_TOP_RGBA)
        glPopMatrix()

        bumper_depth = px_to_meters(max(6.0, robot.length_px * 0.09))
        glPushMatrix()
        glTranslatef(0.0, -depth_m * 0.48 + bumper_depth * 0.5, -height_m * 0.04)
        _draw_box(width_m * 0.92, bumper_depth, height_m * 0.86, ROBOT_BUMPER_RGBA)
        glPopMatrix()

        wheel_w = px_to_meters(max(4.5, robot.width_px * 0.12))
        wheel_d = px_to_meters(max(9.0, robot.length_px * 0.20))
        wheel_h = px_to_meters(max(5.5, robot.width_px * 0.11))
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                glPushMatrix()
                glTranslatef(sx * width_m * 0.48, sy * depth_m * 0.28, -height_m * 0.36)
                _draw_box(wheel_w, wheel_d, wheel_h, ROBOT_WHEEL_RGBA)
                glPopMatrix()

        self._draw_robot_storage(robot, width_m, depth_m, height_m)
        glPopMatrix()

        self._draw_robot_paddle(robot)

    def _draw_robot_storage(self, robot, width_m, depth_m, height_m):
        storage = list(getattr(robot, "storage", []))[:8]
        if not storage:
            return
        spacing_x = width_m * 0.18
        spacing_y = depth_m * 0.18
        start_x = -spacing_x * 1.5
        start_y = -spacing_y * 0.5
        for index, color_name in enumerate(storage):
            col = index % 4
            row = index // 4
            color = BLUE_BLOCK_RGBA if color_name == "blue" else RED_BLOCK_RGBA
            glPushMatrix()
            glTranslatef(start_x + col * spacing_x, start_y + row * spacing_y, height_m * 0.76)
            glScalef(0.36, 0.36, 0.36)
            self._draw_triball(color)
            glPopMatrix()

    def _draw_robot_paddle(self, robot):
        fraction = clamp01(float(getattr(robot, "loader_paddle_fraction", 0.0)))
        requested = bool(getattr(robot, "loader_paddle_requested", False))
        active = requested or fraction > 0.02
        fwd = robot.front_vec()
        right = robot.right_vec()
        front_center = robot.body.position + fwd * (robot.length_px * 0.5)
        center = front_center + fwd * (LOADER_PADDLE_LEN_PX * 0.55 * max(0.08, fraction))
        center_z = 0.052 + ((1.0 - fraction) * 0.195)
        tilt_deg = 88.0 * (1.0 - fraction)
        paddle_w = px_to_meters(LOADER_PADDLE_W_PX)
        paddle_t = px_to_meters(LOADER_PADDLE_T_PX)
        paddle_h = 0.055
        paddle_color = ROBOT_PADDLE_ACTIVE_RGBA if active else ROBOT_PADDLE_IDLE_RGBA
        rod_color = ROBOT_ROD_ACTIVE_RGBA if active else ROBOT_ROD_IDLE_RGBA

        glPushMatrix()
        glTranslatef(px_to_meters(center.x), px_to_meters(center.y), center_z)
        glRotatef(math.degrees(robot.body.angle), 0.0, 0.0, 1.0)
        glRotatef(tilt_deg, 1.0, 0.0, 0.0)
        _draw_box(paddle_w, paddle_t, paddle_h, paddle_color)
        glPopMatrix()

        rod_width = px_to_meters(max(3.0, LOADER_PADDLE_T_PX * 0.42))
        robot_l = front_center - right * (robot.width_px * 0.42)
        robot_r = front_center + right * (robot.width_px * 0.42)
        paddle_l = center - right * (LOADER_PADDLE_W_PX * 0.45)
        paddle_r = center + right * (LOADER_PADDLE_W_PX * 0.45)
        for start, end in ((robot_l, paddle_l), (robot_r, paddle_r)):
            _draw_bar_between(start, end, rod_width, paddle_h * 0.72, 0.050, rod_color)

    def _draw_triball(self, rgba):
        if self._triball_list is None:
            self._triball_list = _build_triball_display_list()
        _set_material(rgba)
        glCallList(self._triball_list)

    def _setup_2d(self):
        width, height = self.size
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(0, width, height, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

    def _draw_surface(self, surface, dest_rect, texture_key):
        texture = self._upload_surface(surface, texture_key)
        x0 = float(dest_rect.x)
        y0 = float(dest_rect.y)
        x1 = float(dest_rect.right)
        y1 = float(dest_rect.bottom)

        glEnable(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, texture)
        glColor4f(1.0, 1.0, 1.0, 1.0)
        glBegin(GL_QUADS)
        glTexCoord2f(0.0, 0.0)
        glVertex2f(x0, y0)
        glTexCoord2f(1.0, 0.0)
        glVertex2f(x1, y0)
        glTexCoord2f(1.0, 1.0)
        glVertex2f(x1, y1)
        glTexCoord2f(0.0, 1.0)
        glVertex2f(x0, y1)
        glEnd()
        glDisable(GL_TEXTURE_2D)

    def _upload_surface(self, surface, texture_key):
        width, height = surface.get_size()
        if hasattr(pygame.image, "tobytes"):
            pixels = pygame.image.tobytes(surface, "RGBA", False)
        else:
            pixels = pygame.image.tostring(surface, "RGBA", False)

        texture_info = self._textures.get(texture_key)
        if texture_info is None:
            texture = glGenTextures(1)
            texture_size = None
            self._textures[texture_key] = [texture, texture_size]
            glBindTexture(GL_TEXTURE_2D, texture)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        else:
            texture, texture_size = texture_info

        glBindTexture(GL_TEXTURE_2D, texture)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        if texture_size != (width, height):
            glTexImage2D(
                GL_TEXTURE_2D,
                0,
                GL_RGBA,
                width,
                height,
                0,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                pixels,
            )
            self._textures[texture_key][1] = (width, height)
        else:
            glTexSubImage2D(
                GL_TEXTURE_2D,
                0,
                0,
                0,
                width,
                height,
                GL_RGBA,
                GL_UNSIGNED_BYTE,
                pixels,
            )
        return texture


def _rgba_for_shape(shape, long_ids, middle_ids):
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


def _set_color(rgba):
    glColor4f(float(rgba[0]), float(rgba[1]), float(rgba[2]), float(rgba[3]))


def clamp01(value):
    return 0.0 if value < 0.0 else 1.0 if value > 1.0 else value


def _set_material(rgba):
    _set_color(rgba)
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, list(rgba))


def _quad_xy(x0, y0, x1, y1, z):
    glBegin(GL_QUADS)
    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(x0, y0, z)
    glVertex3f(x1, y0, z)
    glVertex3f(x1, y1, z)
    glVertex3f(x0, y1, z)
    glEnd()


def _draw_rect_on_floor(rect, rgba, z=0.005):
    _set_color(rgba)
    _quad_xy(
        px_to_meters(rect.left),
        px_to_meters(rect.top),
        px_to_meters(rect.right),
        px_to_meters(rect.bottom),
        z,
    )


def _draw_parking_floor_marks():
    outer = inches_to_px(PARK_ZONE_OUTER_SIZE_IN)
    half = outer * 0.5
    blue = pygame.Rect(int((FIELD_SIZE_PX * 0.5) - half), 0, int(outer), int(outer))
    red = pygame.Rect(int((FIELD_SIZE_PX * 0.5) - half), int(FIELD_SIZE_PX - outer), int(outer), int(outer))
    _draw_rect_on_floor(blue, (0.08, 0.36, 0.78, 0.18), z=0.004)
    _draw_rect_on_floor(red, (0.82, 0.10, 0.10, 0.18), z=0.004)


def _loader_inward_angle(loader):
    return math.pi * 0.5 if loader.pos.y < (FIELD_SIZE_PX * 0.5) else -math.pi * 0.5


def _draw_box(width, depth, height, rgba):
    _set_material(rgba)
    hx = width * 0.5
    hy = depth * 0.5
    hz = height * 0.5

    glBegin(GL_QUADS)
    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(-hx, -hy, hz)
    glVertex3f(hx, -hy, hz)
    glVertex3f(hx, hy, hz)
    glVertex3f(-hx, hy, hz)

    glNormal3f(0.0, 0.0, -1.0)
    glVertex3f(-hx, hy, -hz)
    glVertex3f(hx, hy, -hz)
    glVertex3f(hx, -hy, -hz)
    glVertex3f(-hx, -hy, -hz)

    glNormal3f(0.0, -1.0, 0.0)
    glVertex3f(-hx, -hy, -hz)
    glVertex3f(hx, -hy, -hz)
    glVertex3f(hx, -hy, hz)
    glVertex3f(-hx, -hy, hz)

    glNormal3f(0.0, 1.0, 0.0)
    glVertex3f(-hx, hy, hz)
    glVertex3f(hx, hy, hz)
    glVertex3f(hx, hy, -hz)
    glVertex3f(-hx, hy, -hz)

    glNormal3f(1.0, 0.0, 0.0)
    glVertex3f(hx, -hy, -hz)
    glVertex3f(hx, hy, -hz)
    glVertex3f(hx, hy, hz)
    glVertex3f(hx, -hy, hz)

    glNormal3f(-1.0, 0.0, 0.0)
    glVertex3f(-hx, -hy, hz)
    glVertex3f(-hx, hy, hz)
    glVertex3f(-hx, hy, -hz)
    glVertex3f(-hx, -hy, -hz)
    glEnd()


def _draw_cylinder(radius, height, rgba, segments=28):
    _set_material(rgba)
    half_h = height * 0.5
    glBegin(GL_QUAD_STRIP)
    for i in range(segments + 1):
        theta = math.tau * (i / segments)
        x = math.cos(theta) * radius
        y = math.sin(theta) * radius
        glNormal3f(math.cos(theta), math.sin(theta), 0.0)
        glVertex3f(x, y, -half_h)
        glVertex3f(x, y, half_h)
    glEnd()

    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0.0, 0.0, 1.0)
    glVertex3f(0.0, 0.0, half_h)
    for i in range(segments + 1):
        theta = math.tau * (i / segments)
        glVertex3f(math.cos(theta) * radius, math.sin(theta) * radius, half_h)
    glEnd()

    glBegin(GL_TRIANGLE_FAN)
    glNormal3f(0.0, 0.0, -1.0)
    glVertex3f(0.0, 0.0, -half_h)
    for i in range(segments, -1, -1):
        theta = math.tau * (i / segments)
        glVertex3f(math.cos(theta) * radius, math.sin(theta) * radius, -half_h)
    glEnd()


def _draw_cylinder_wall_with_lower_gap(
    radius,
    height,
    rgba,
    *,
    gap_angle,
    gap_width,
    gap_height,
    segments=48,
):
    _set_material(rgba)
    glBegin(GL_QUADS)
    for i in range(segments):
        a0 = math.tau * (i / segments)
        a1 = math.tau * ((i + 1) / segments)
        mid = 0.5 * (a0 + a1)
        z0 = gap_height if _angle_in_gap(mid, gap_angle, gap_width) else 0.0
        if z0 >= height:
            continue

        x0 = math.cos(a0) * radius
        y0 = math.sin(a0) * radius
        x1 = math.cos(a1) * radius
        y1 = math.sin(a1) * radius
        nx = math.cos(mid)
        ny = math.sin(mid)
        glNormal3f(nx, ny, 0.0)
        glVertex3f(x0, y0, z0)
        glVertex3f(x1, y1, z0)
        glVertex3f(x1, y1, height)
        glVertex3f(x0, y0, height)
    glEnd()


def _draw_flat_ring(outer_radius, inner_radius, z, rgba, *, gap_angle=None, gap_width=0.0, segments=56):
    _set_material(rgba)
    glBegin(GL_QUADS)
    for i in range(segments):
        a0 = math.tau * (i / segments)
        a1 = math.tau * ((i + 1) / segments)
        mid = 0.5 * (a0 + a1)
        if gap_angle is not None and _angle_in_gap(mid, gap_angle, gap_width):
            continue

        glNormal3f(0.0, 0.0, 1.0)
        glVertex3f(math.cos(a0) * inner_radius, math.sin(a0) * inner_radius, z)
        glVertex3f(math.cos(a0) * outer_radius, math.sin(a0) * outer_radius, z)
        glVertex3f(math.cos(a1) * outer_radius, math.sin(a1) * outer_radius, z)
        glVertex3f(math.cos(a1) * inner_radius, math.sin(a1) * inner_radius, z)
    glEnd()


def _angle_in_gap(angle, gap_angle, gap_width):
    return abs(_angle_delta(angle, gap_angle)) <= gap_width * 0.5


def _angle_delta(a, b):
    return ((a - b + math.pi) % math.tau) - math.pi


def _draw_shadow_disc(x, y, radius):
    _set_color(SHADOW_RGBA)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(x, y, 0.008)
    for i in range(33):
        theta = math.tau * (i / 32)
        glVertex3f(x + math.cos(theta) * radius, y + math.sin(theta) * radius * 0.72, 0.008)
    glEnd()


def _draw_bar_between(start, end, width, height, z_center, rgba):
    dx = end.x - start.x
    dy = end.y - start.y
    length_px = math.hypot(dx, dy)
    if length_px <= 1e-6:
        return
    cx = (start.x + end.x) * 0.5
    cy = (start.y + end.y) * 0.5
    angle = math.atan2(dy, dx)
    glPushMatrix()
    glTranslatef(px_to_meters(cx), px_to_meters(cy), z_center)
    glRotatef(math.degrees(angle), 0.0, 0.0, 1.0)
    _draw_box(px_to_meters(length_px), width, height, rgba)
    glPopMatrix()


def _apply_body_transform(body):
    glTranslatef(px_to_meters(body.position.x), px_to_meters(body.position.y), body.z)
    glMultMatrixf(_quat_to_matrix(body.quat))


def _quat_to_matrix(quat):
    x, y, z, w = [float(v) for v in quat]
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return [
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy + wz),
        2.0 * (xz - wy),
        0.0,
        2.0 * (xy - wz),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz + wx),
        0.0,
        2.0 * (xz + wy),
        2.0 * (yz - wx),
        1.0 - 2.0 * (xx + yy),
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def _size_from_vertices(shape):
    verts = list(getattr(shape, "local_vertices", []))
    if not verts:
        return (10.0, 10.0)
    min_x = min(v.x for v in verts)
    max_x = max(v.x for v in verts)
    min_y = min(v.y for v in verts)
    max_y = max(v.y for v in verts)
    return (max(1.0, max_x - min_x), max(1.0, max_y - min_y))


def _build_triball_display_list():
    vertices, triangles = _build_triball_mesh()
    list_id = glGenLists(1)
    glNewList(list_id, GL_COMPILE)
    glBegin(GL_TRIANGLES)
    for a_idx, b_idx, c_idx in triangles:
        a = vertices[a_idx]
        b = vertices[b_idx]
        c = vertices[c_idx]
        nx, ny, nz = _triangle_normal(a, b, c)
        glNormal3f(nx, ny, nz)
        glVertex3f(*a)
        glVertex3f(*b)
        glVertex3f(*c)
    glEnd()
    glEndList()
    return list_id


def _build_triball_mesh():
    half_w = TRIBALL_MESH.width_m * 0.5
    half_d = TRIBALL_MESH.depth_m * 0.5
    half_h = TRIBALL_MESH.height_m * 0.5

    vertices = []
    for z_norm, radius_scale in TRIBALL_RINGS:
        z = z_norm * half_h
        for i in range(TRIBALL_SIDES):
            theta = math.tau * i / TRIBALL_SIDES
            facet_ratio = 1.0 if i % 2 == 0 else TRIBALL_FLAT_RATIO
            vertices.append((
                math.cos(theta) * half_w * radius_scale * facet_ratio,
                math.sin(theta) * half_d * radius_scale * facet_ratio,
                z,
            ))

    triangles = []
    ring_count = len(TRIBALL_RINGS)
    for ring in range(ring_count - 1):
        start = ring * TRIBALL_SIDES
        next_start = (ring + 1) * TRIBALL_SIDES
        for i in range(TRIBALL_SIDES):
            a = start + i
            b = start + ((i + 1) % TRIBALL_SIDES)
            c = next_start + ((i + 1) % TRIBALL_SIDES)
            d = next_start + i
            triangles.append((a, b, c))
            triangles.append((a, c, d))

    bottom_center = len(vertices)
    vertices.append((0.0, 0.0, -half_h))
    top_center = len(vertices)
    vertices.append((0.0, 0.0, half_h))
    top_start = (ring_count - 1) * TRIBALL_SIDES
    for i in range(TRIBALL_SIDES):
        a = i
        b = (i + 1) % TRIBALL_SIDES
        triangles.append((bottom_center, b, a))
        ta = top_start + i
        tb = top_start + ((i + 1) % TRIBALL_SIDES)
        triangles.append((top_center, ta, tb))

    return vertices, triangles


def _triangle_normal(a, b, c):
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = (uy * vz) - (uz * vy)
    ny = (uz * vx) - (ux * vz)
    nz = (ux * vy) - (uy * vx)
    mag = math.sqrt((nx * nx) + (ny * ny) + (nz * nz))
    if mag <= 1e-9:
        return (0.0, 0.0, 1.0)
    return (nx / mag, ny / mag, nz / mag)
