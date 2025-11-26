# -*- coding: utf-8 -*-
"""
SIMPLIFIED CAR MODEL - Clear Names, Simple Physics
"""

import os
from PIL import Image, ImageTk
import math
from config import *

car_img_path = "car1.png" if os.path.exists("car1.png") else "slotcar_track_sim/car1.png"
car1_img = Image.open(car_img_path)


class Car:
    def __init__(self, x, y, b, img, name, piecewise_curvature, piecewise_angle, piecewise_position, parameters):
        self.fi = img
        self.name = name
        self.img = None

        # Car geometry
        self.a = 0.020  # Distance CG to rear (m)
        self.b = 0.025  # Distance CG to front/guide pin (m)

        # State variables
        self.s = 0  # Position along track (m)
        self.v = 0  # Velocity (m/s)
        self.slip_angle = 0  # Drift angle (radians)

        # Visual state
        self.x = x
        self.y = y
        self.b_heading = b

        # Parameters
        self.voltage = parameters["voltage"]
        self.mass = parameters["mass"]  # grams
        self.mu_static = parameters["static_f"]  # μ_s
        self.mu_dynamic = parameters["dynamic_f"]  # μ_d
        self.magnet_strength = parameters["max_energy"]  # MGOe
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]
        self.R_motor = 0.5  # Motor resistance (Ohms)

        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

        self.derailed = False

    # ============== FORCE CALCULATIONS ==============

    def calculate_F_motor(self):
        """
        Motor force along car body
        Formula: F = (η * N * k_t / (r * R)) * (V - k_e * N * v / r)
        """
        r = self.wheel_r / 1000  # mm to m
        eta = self.gear_efficiency / 100
        N = self.gear_ratio
        k_t = self.torque_c * 0.001
        k_e = self.back_emf_c * 0.001

        back_emf = (k_e * N * self.v) / r

        F = (eta * N * k_t / (r * self.R_motor)) * (self.voltage - back_emf)
        return F

    def calculate_N_total(self):
        """
        Total normal force (perpendicular to track surface)
        Formula: N = m*g + F_magnet
        """
        mass_kg = self.mass / 1000
        g = 9.81

        # Weight
        N_weight = mass_kg * g

        # Magnet adds downforce
        F_magnet_down = self.magnet_strength * 0.05  # MGOe to Newtons (simplified)

        N_total = N_weight + F_magnet_down
        return N_total

    def calculate_F_centrifugal(self):
        """
        Centrifugal force (apparent outward force in rotating frame)
        Formula: F = m * v² / R
        Direction: Perpendicular to velocity, away from curve center
        """
        if self.v == 0:
            return 0

        curvature = self.piecewise_curvature.get(self.s)

        if abs(curvature) < 0.0001:  # Straight section
            return 0

        R = abs(1 / curvature)
        mass_kg = self.mass / 1000

        F_centrifugal = mass_kg * (self.v**2) / R
        return F_centrifugal

    def calculate_F_static_lateral_max(self):
        """
        Maximum lateral grip BEFORE slip starts (threshold)
        Formula: F_max = μ_s * N
        This is NOT a force that acts - it's a LIMIT
        """
        N = self.calculate_N_total()
        F_max = self.mu_static * N
        return F_max

    def calculate_F_dynamic_lateral(self):
        """
        Lateral friction DURING slip (actual force)
        Formula: F = μ_d * N
        Direction: Toward curve center
        """
        N = self.calculate_N_total()
        F_friction = self.mu_dynamic * N
        return F_friction

    def calculate_F_magnet_restoring(self):
        """
        SIMPLIFIED magnetic restoring force
        Magnet wants to be above rail. If offset, pulls back.

        Formula: F = -k * offset (linear spring, up to limit)
        k = magnet_strength * 10

        At offset > 0.015m (15mm): F = 0 (too far, no magnetic effect)
        """
        # Spring constant depends on magnet strength
        k = self.magnet_strength * 10  # N/m

        # Lateral offset (how far rear is from centerline)
        # We approximate from slip angle: offset ≈ a * sin(slip_angle)
        offset = self.a * math.sin(self.slip_angle)

        # Linear restoring force
        if abs(offset) < 0.015:  # Within 15mm
            F_restore = -k * offset
        else:
            F_restore = 0  # Too far, no magnetic pull

        return F_restore

    # ============== PHYSICS TICK ==============

    def tick(self, deltat):
        """
        Main physics simulation
        """
        if self.derailed:
            # Spinning Animation
            self.b_heading += 0.33 * self.v
            self.b_heading %= 6.28
            self.v *= 0.97
            self.x += math.cos(self.piecewise_angle.get(self.s)) * self.v * deltat
            self.y += math.sin(self.piecewise_angle.get(self.s)) * self.v * deltat
            if self.v < 0.01:
                self.v = 0
            return

        mass_kg = self.mass / 1000

        # === STEP 1: Calculate all forces ===

        F_motor = self.calculate_F_motor()  # Along car body
        F_centrifugal = self.calculate_F_centrifugal()  # Outward in curve
        F_max_static = self.calculate_F_static_lateral_max()  # Threshold

        # === STEP 2: Check if slipping laterally ===

        is_slipping = F_centrifugal > F_max_static

        if not is_slipping:
            # NO SLIP: Grip holds, friction adapts to match centrifugal
            # Slip angle damps back to zero
            self.slip_angle = self.slip_angle * 0.9 if self.slip_angle > 0.01 else 0

        else:
            # SLIP: Over threshold, rear slides outward
            F_friction_lateral = self.calculate_F_dynamic_lateral()
            F_magnet_restore = self.calculate_F_magnet_restoring()

            # Net lateral force
            F_net_lateral = F_centrifugal - F_friction_lateral + F_magnet_restore

            # This creates torque about guide pin: τ = F * distance
            # τ = F_net_lateral * self.a (distance to rear)
            # Angular acceleration = τ / I (moment of inertia)
            # Simplified: slip_rate proportional to F_net / mass

            slip_rate = F_net_lateral / (mass_kg * 10) if self.v > 0 else 0  # rad/s

            # Limit rate
            MAX_SLIP_RATE = 2.0
            slip_rate = max(-MAX_SLIP_RATE, min(MAX_SLIP_RATE, slip_rate))

            self.slip_angle += slip_rate * deltat

            # Clamp slip angle
            MAX_SLIP = math.radians(50)
            self.slip_angle = max(-MAX_SLIP, min(MAX_SLIP, self.slip_angle))

        # === STEP 3: Effective forward force ===

        # F_motor acts along car body
        # Velocity is along track
        # Component in velocity direction: cos(slip_angle)

        cos_alpha = math.cos(self.slip_angle)
        if cos_alpha < 0:
            cos_alpha = 0

        F_effective_forward = F_motor * cos_alpha

        # === STEP 4: Update velocity ===
        MAX_ACCEL = 30
        acceleration = F_effective_forward / mass_kg
        acceleration = max(-MAX_ACCEL, min(MAX_ACCEL, acceleration))

        MAX_V = 15
        self.v += acceleration * deltat
        self.v = max(0, min(MAX_V, self.v))

        # === STEP 5: Update position ===

        self.s += self.v * deltat

        # === STEP 6: Derailment check ===

        if abs(self.slip_angle) >= math.radians(42):
            self.derailed = True
            print(f"DERAILED at slip_angle = {math.degrees(self.slip_angle):.1f}°")
            return

        # === STEP 7: Visual position ===

        track_angle = self.piecewise_angle.get(self.s)
        pin_x, pin_y = self.piecewise_position.get(self.s)

        self.b_heading = track_angle + self.slip_angle

        # CG is behind guide pin
        self.x = pin_x - self.b * math.cos(self.b_heading)
        self.y = pin_y - self.b * math.sin(self.b_heading)

    # ============== DRAWING ==============

    def draw(self, canvas):
        if self.img:
            canvas.delete(self.img)

        ow, oh = self.fi.size
        screen_x, screen_y = m_to_px(canvas, self.x, self.y)
        rot_angle = self.b_heading * 180 / math.pi

        img = self.fi.rotate(-90 + rot_angle, resample=Image.BICUBIC)
        img = img.resize((int(ow * SCALE), int(oh * SCALE)), Image.Resampling.LANCZOS)

        self.photo = ImageTk.PhotoImage(img)
        self.img = canvas.create_image(screen_x, screen_y, image=self.photo)

        # Debug info
        canvas.create_rectangle(5, 5, 300, 180, fill="black", outline="white", width=2)

        status = "DERAILED!" if self.derailed else ("SLIPPING" if abs(self.slip_angle) > 0.05 else "Grip OK")
        color = "red" if self.derailed else ("orange" if abs(self.slip_angle) > 0.05 else "lime")

        canvas.create_text(15, 20, anchor="w", text=f"Status: {status}", fill=color, font=("Arial", 14, "bold"))
        canvas.create_text(
            15,
            50,
            anchor="w",
            text=f"Slip Angle: {math.degrees(self.slip_angle):.1f}°",
            fill="white",
            font=("Arial", 11),
        )
        canvas.create_text(15, 75, anchor="w", text=f"Velocity: {self.v:.2f} m/s", fill="white", font=("Arial", 11))

        F_motor = self.calculate_F_motor()
        F_eff = F_motor * math.cos(self.slip_angle)
        canvas.create_text(15, 100, anchor="w", text=f"F_motor: {F_motor:.2f} N", fill="white", font=("Arial", 11))
        canvas.create_text(15, 125, anchor="w", text=f"F_eff_forward: {F_eff:.2f} N", fill="cyan", font=("Arial", 11))
        canvas.create_text(
            15,
            150,
            anchor="w",
            text=f"F_centrifugal: {self.calculate_F_centrifugal():.2f} N",
            fill="red",
            font=("Arial", 11),
        )

    def updateParameters(self, parameters: dict) -> None:
        self.voltage = parameters["voltage"]
        self.mass = parameters["mass"]
        self.mu_static = parameters["static_f"]
        self.mu_dynamic = parameters["dynamic_f"]
        self.magnet_strength = parameters["max_energy"]
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]
