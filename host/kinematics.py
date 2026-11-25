"""Mural's kinematic model, ported to Python from the upstream ESP32 firmware.

Source: nikivanov/mural, src/movement.cpp (Movement::getBeltLengths and helpers).
See the upstream KinematicModel.md for the derivation. Upstream is licensed
CC BY-NC 4.0; this port is under the same terms.
"""

import math
from dataclasses import dataclass

# Geometry and physics constants (same values as upstream movement.h).
DEFAULT_PULLEY_DIAMETER = 10.14926   # mm, effective diameter of pulley + belt
MID_PULLEY_TO_WALL = 41.0            # mm
HOMED_STEP_OFFSET_MM = 40.0          # mm of belt left when fully retracted against the stop screw
MASS_BOT = 0.55                      # kg
G = 9.81                             # m/s^2
PULLEY_AXIS_DISTANCE = 89.50         # mm
AXES_TO_CENTER_OF_MASS = 10.0        # mm
BELT_ELONGATION_COEFFICIENT = 5e-5   # m/N
HOME_Y_OFFSET_MM = 350
SAFE_Y_FRACTION = 0.25
SAFE_X_FRACTION = 0.2

PRINT_SPEED_STEPS = 650
MOVE_SPEED_STEPS = 2000


@dataclass
class Geometry:
    top_distance: float = -1          # d_pins, mm
    pulley_diameter: float = DEFAULT_PULLEY_DIAMETER
    steps_per_rotation: int = 200 * 8  # full steps * microsteps

    def __post_init__(self):
        self.gamma_last = 0.0
        self.update_pulley_geometry()
        if self.top_distance > 0:
            self.set_top_distance(self.top_distance)

    # --- setup -----------------------------------------------------------

    def update_pulley_geometry(self):
        self.circumference = self.pulley_diameter * math.pi
        q = self.pulley_diameter / (2.0 * math.sqrt(2.0))
        self.d_t = PULLEY_AXIS_DISTANCE - 2.0 * q
        self.d_p = q
        self.d_m = AXES_TO_CENTER_OF_MASS + q

    def set_pulley_diameter(self, d):
        self.pulley_diameter = d
        self.update_pulley_geometry()

    def set_top_distance(self, distance):
        self.top_distance = distance
        self.min_safe_y = SAFE_Y_FRACTION * distance
        self.min_safe_x_offset = SAFE_X_FRACTION * distance
        self.width = distance - 2 * self.min_safe_x_offset

    def home_coordinates(self):
        if self.top_distance == -1:
            return (0.0, 0.0)
        return (self.width / 2, float(HOME_Y_OFFSET_MM))

    def mm_to_steps(self, mm):
        return int((mm / self.circumference) * self.steps_per_rotation)

    def homed_steps_offset(self):
        return self.mm_to_steps(HOMED_STEP_OFFSET_MM)

    # --- model -----------------------------------------------------------

    def _left_tangent(self, fx, fy, gamma):
        s = self.d_t / 2.0
        px = s * math.cos(gamma) - self.d_p * math.sin(gamma)
        py = s * math.sin(gamma) + self.d_p * math.cos(gamma)
        return fx - px, fy - py

    def _right_tangent(self, fx, fy, gamma):
        s = self.d_t / 2.0
        px = s * math.cos(gamma) + self.d_p * math.sin(gamma)
        py = s * math.sin(gamma) - self.d_p * math.cos(gamma)
        return fx + px, fy + py

    def _belt_angles(self, fx, fy, gamma):
        xl, yl = self._left_tangent(fx, fy, gamma)
        xr, yr = self._right_tangent(fx, fy, gamma)
        return math.atan2(yl, xl), math.atan2(yr, self.top_distance - xr)

    @staticmethod
    def _belt_forces(phi_l, phi_r):
        fg = MASS_BOT * G
        f_r = fg * math.cos(phi_l) / math.sin(phi_l + phi_r)
        f_l = fg * math.cos(phi_r) / math.sin(phi_l + phi_r)
        return f_l, f_r

    def _solve_torque(self, phi_l, phi_r, f_l, f_r, gamma_init):
        s = self.d_t / 2.0
        step = 0.20 * math.pi / 180.0
        g_min = -90.0 * math.pi / 180.0
        g_max = 90.0 * math.pi / 180.0
        window = 2.0 * math.pi / 180.0
        best_gamma, best_t = 99999999.0, 99999999.0
        fg = MASS_BOT * G
        gamma = gamma_init - window
        while g_min < gamma < g_max and gamma <= gamma_init + window:
            t_l = s * math.sin(phi_l - gamma) * f_l
            t_r = s * math.sin(phi_r + gamma) * f_r
            t_m = self.d_m * math.tan(gamma) * fg * math.cos(gamma)
            t_delta = t_r - t_l + t_m
            if abs(t_delta) < abs(best_t):
                best_t, best_gamma = t_delta, gamma
            else:
                return best_gamma
            gamma += step
        return best_gamma

    def belt_lengths_steps(self, x, y):
        """Image coordinates (mm) -> (left, right) motor positions in steps."""
        fx = x + self.min_safe_x_offset
        fy = y + self.min_safe_y
        gamma = self.gamma_last
        phi_l = phi_r = f_l = f_r = 0.0
        for _ in range(20):
            phi_l, phi_r = self._belt_angles(fx, fy, gamma)
            f_l, f_r = self._belt_forces(phi_l, phi_r)
            last = gamma
            gamma = self._solve_torque(phi_l, phi_r, f_l, f_r, gamma)
            if abs(last - gamma) < 0.25 / 180.0 * math.pi:
                break
        self.gamma_last = gamma

        lx, ly = self._left_tangent(fx, fy, gamma)
        rx, ry = self._right_tangent(fx, fy, gamma)
        left_flat = math.hypot(lx, ly)
        right_flat = math.hypot(self.top_distance - rx, ry)
        left = math.hypot(left_flat, MID_PULLEY_TO_WALL)
        right = math.hypot(right_flat, MID_PULLEY_TO_WALL)
        left /= 1 + BELT_ELONGATION_COEFFICIENT * f_l
        right /= 1 + BELT_ELONGATION_COEFFICIENT * f_r
        return self.mm_to_steps(left), self.mm_to_steps(right)
