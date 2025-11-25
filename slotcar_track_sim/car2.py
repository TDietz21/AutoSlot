# -*- coding: utf-8 -*-

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

        # Car geometry (derived from image)
        # Typical slot car image: top = front, bottom = rear, center = CG
        img_width, img_height = img.size

        # In meters (slot car scale)
        self.car_length = img_height * 0.001  # pixels to meters (rough)
        self.guide_pin_distance = self.car_length * 0.5  # Pin at front (top of image)
        self.rear_distance = self.car_length * 0.5  # Rear at bottom of image

        # For now, use typical slot car values
        self.guide_pin_distance = 0.040  # 20mm in front of CG
        self.rear_distance = 0.025  # 25mm behind CG

        # Internal State - TRACK COORDINATES
        self.s = 0  # distance along track centerline (meters)
        self.v = 0  # velocity along track (m/s)

        # Internal State - CAR ORIENTATION
        # The guide pin position follows the track perfectly (s)
        # The car body can rotate around the guide pin
        self.slip_angle = 0  # angle between car heading and velocity direction (radians)

        # Visual state (calculated from s and slip_angle)
        self.x = x  # CG x position
        self.y = y  # CG y position
        self.b = b  # car heading angle

        # Inputs
        self.w = 0  # wheel angle (not used for slot cars - guide pin steers)
        self.iv = parameters["voltage"]

        # Parameters
        self.magnet_go = parameters["max_energy"]
        self.mass = parameters["mass"]  # in GRAMS
        self.static_f = parameters["static_f"]
        self.dynamic_f = parameters["dynamic_f"]
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.back_emf = parameters["back_emf"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]
        self.R = 0.5  # internal motor resistance (ohms)

        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

        # Derailment
        self.derailed = False

    def calculate_motor_force(self, voltage, velocity):
        """
        Calculate motor force with gearbox (slide 12)
        F(V, v) = (η * N * k_t / (r * R)) * (V - (k_e * N * v) / r)
        """
        r = self.wheel_r / 1000  # mm to meters
        eta = self.gear_efficiency / 100
        N = self.gear_ratio
        k_t = self.torque_c * 0.01
        k_e = self.back_emf_c * 0.01
        R = self.R

        back_emf_term = (k_e * N * velocity) / r
        force = (eta * N * k_t / (r * R)) * (voltage - back_emf_term)

        return force

    def calculate_centrifugal_force(self):
        """
        Centrifugal force at the REAR of the car
        F = m * v² / R

        This acts at distance 'rear_distance' from CG
        """
        if self.v == 0:
            return 0

        curvature = self.piecewise_curvature.get(self.s)

        if abs(curvature) < 0.0001:  # straight
            return 0

        R = abs(1 / curvature)
        mass_kg = self.mass / 1000

        F_centrifugal = mass_kg * (self.v ** 2) / R

        return F_centrifugal

    def calculate_max_lateral_grip(self):
        """
        Maximum lateral grip before sliding
        F_max = μ_s * (m*g + F_magnet)
        """
        mass_kg = self.mass / 1000
        g = 9.81

        # Base normal force
        N_base = mass_kg * g

        # Magnet adds downforce (simplified model)
        F_magnet_down = self.magnet_go * 0.05  # MGOe to Newtons

        N_total = N_base + F_magnet_down

        # Static friction threshold
        F_max = self.static_f * N_total

        return F_max

    def calculate_lateral_friction(self):
        """
        Actual lateral friction force when sliding
        F = μ_d * (m*g + F_magnet)
        """
        mass_kg = self.mass / 1000
        g = 9.81

        N_base = mass_kg * g
        F_magnet_down = self.magnet_go * 0.05
        N_total = N_base + F_magnet_down

        F_friction = self.dynamic_f * N_total

        return F_friction

    def tick(self, deltat):
        """
        CLEAN PHYSICS SIMULATION

        Key concepts:
        1. Guide pin follows track perfectly (position s)
        2. Motor provides forward force along car heading
        3. Centrifugal force tries to push rear outward
        4. Friction resists lateral motion
        5. If centrifugal > grip → rear slides → slip angle grows
        6. Forward force reduced by cos(slip_angle)
        """

        if self.derailed:
            # Spinning animation when derailed
            self.b += 2.0 * deltat  # Rotate fast
            self.v *= 0.95  # Slow down

            # Move off track
            self.x += math.cos(self.b) * 0.5 * deltat
            self.y += math.sin(self.b) * 0.5 * deltat

            if self.v < 0.01:
                self.v = 0
            return

        mass_kg = self.mass / 1000

        # === STEP 1: CALCULATE FORCES ===

        # Motor force (always acts along car heading)
        F_motor = self.calculate_motor_force(self.iv, self.v)

        # CRITICAL FIX: Prevent negative motor force causing oscillation
        if F_motor < 0:
            F_motor = 0

        # Centrifugal force (tries to push rear outward)
        F_centrifugal = self.calculate_centrifugal_force()

        # Maximum lateral grip available
        F_max_lateral_grip = self.calculate_max_lateral_grip()

        # === STEP 2: CHECK IF REAR IS SLIDING ===

        is_sliding = F_centrifugal > F_max_lateral_grip

        if not is_sliding:
            # === NO SLIDE: Perfect grip ===
            # Friction perfectly balances centrifugal force
            # Slip angle gradually returns to zero

            self.slip_angle *= 0.8  # Smooth damping

        else:
            # === SLIDE: Rear is slipping laterally ===
            # Friction can't fully resist centrifugal force
            # Net lateral force causes slip angle to grow

            F_lateral_friction = self.calculate_lateral_friction()  # μ_d * N

            # Net lateral force at rear
            F_net_lateral = F_centrifugal - F_lateral_friction

            # This creates a torque around the guide pin (slide 16)
            # τ = F * distance
            # α_angular = τ / I
            # For simplicity, we model slip angle rate directly

            # Slip angle rate (radians per second)
            # Larger net force = faster slip growth
            slip_rate = F_net_lateral / (mass_kg * 10)  # Simplified dynamics

            # Update slip angle with clamping
            delta_slip = slip_rate * deltat
            self.slip_angle += delta_slip

            # Limit slip angle
            MAX_SLIP = math.radians(50)
            self.slip_angle = max(-MAX_SLIP, min(MAX_SLIP, self.slip_angle))

        # === STEP 3: EFFECTIVE FORWARD FORCE (slide 16) ===

        # Motor force acts along car heading
        # But velocity is along track direction
        # So effective forward force = F_motor * cos(slip_angle)

        # CRITICAL FIX: Use abs() to prevent sign issues
        cos_slip = math.cos(self.slip_angle)
        F_effective_forward = F_motor * abs(cos_slip)

        # Additional safety: minimum force threshold
        if F_effective_forward < 0.001:
            F_effective_forward = 0

        # === STEP 4: UPDATE VELOCITY ===

        acceleration = F_effective_forward / mass_kg

        # CRITICAL FIX: Clamp acceleration to reasonable values
        MAX_ACCEL = 50  # m/s²
        acceleration = max(-MAX_ACCEL, min(MAX_ACCEL, acceleration))

        self.v += acceleration * deltat

        # CRITICAL FIX: Never negative velocity
        if self.v < 0:
            self.v = 0

        # CRITICAL FIX: Maximum velocity cap (prevent runaway)
        MAX_VELOCITY = 20  # m/s
        if self.v > MAX_VELOCITY:
            self.v = MAX_VELOCITY

        # === STEP 5: UPDATE POSITION ALONG TRACK ===

        # Velocity is along track direction
        self.s += self.v * deltat

        # === STEP 6: DERAILMENT CHECK ===

        if abs(self.slip_angle) > math.radians(50):
            if not self.derailed:
                self.derailed = True
                print(f"DERAILED! Slip angle = {math.degrees(self.slip_angle):.1f}°")
            return

        # === STEP 7: CALCULATE VISUAL POSITION ===

        # Guide pin position (follows track perfectly)
        track_angle = self.piecewise_angle.get(self.s)
        pin_x, pin_y = self.piecewise_position.get(self.s)

        # Car heading = track angle + slip angle
        self.b = track_angle + self.slip_angle

        # CG is behind the guide pin
        self.x = pin_x - self.guide_pin_distance * math.cos(self.b)
        self.y = pin_y - self.guide_pin_distance * math.sin(self.b)

    def draw(self, canvas):
        if self.img:
            canvas.delete(self.img)

        ow, oh = self.fi.size
        screen_x, screen_y = m_to_px(canvas, self.x, self.y)
        rot_angle = self.b * 180 / math.pi

        img = self.fi.rotate(-90 + (rot_angle), resample=Image.BICUBIC)
        img = img.resize((int(ow * SCALE), int(oh * SCALE)), Image.Resampling.LANCZOS)

        self.photo = ImageTk.PhotoImage(img)
        self.img = canvas.create_image(screen_x, screen_y, image=self.photo)

        # === DEBUG INFO ===
        canvas.create_rectangle(5, 5, 280, 160, fill='black', outline='white', width=2)

        # Status
        is_sliding = abs(self.slip_angle) > math.radians(2)  # 2 degree threshold
        status_color = "red" if self.derailed else ("orange" if is_sliding else "lime")
        status_text = "DERAILED!" if self.derailed else ("SLIDING" if is_sliding else "Grip OK")
        canvas.create_text(15, 20, anchor="w", text=f"Status: {status_text}",
                           fill=status_color, font=("Arial", 14, "bold"))

        # Slip angle
        slip_deg = self.slip_angle * 180 / math.pi
        canvas.create_text(15, 50, anchor="w",
                           text=f"Slip Angle: {slip_deg:.1f}°",
                           fill="white", font=("Arial", 12))

        # Velocity
        canvas.create_text(15, 80, anchor="w",
                           text=f"Velocity: {self.v:.2f} m/s",
                           fill="white", font=("Arial", 12))

        # Motor force
        F_motor = self.calculate_motor_force(self.iv, self.v)
        canvas.create_text(15, 110, anchor="w",
                           text=f"Motor Force: {F_motor:.1f} N",
                           fill="white", font=("Arial", 12))

        # Effective forward
        F_eff = F_motor * math.cos(self.slip_angle)
        canvas.create_text(15, 140, anchor="w",
                           text=f"Eff. Forward: {F_eff:.1f} N",
                           fill="cyan", font=("Arial", 12))

    def updateParameters(self, parameters: dict) -> None:
        self.iv = parameters["voltage"]
        self.magnet_go = parameters["max_energy"]
        self.mass = parameters["mass"]
        self.static_f = parameters["static_f"]
        self.dynamic_f = parameters["dynamic_f"]
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.back_emf = parameters["back_emf"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]