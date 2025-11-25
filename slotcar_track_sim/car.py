# -*- coding: utf-8 -*-

import os
from typing import Literal
from PIL import Image, ImageTk
import math
from config import *

DRIVING_STATE = Literal["driving", "derailed"]

car_img_path = "car1.png" if os.path.exists("car1.png") else "slotcar_track_sim/car1.png"
car1_img = Image.open(car_img_path)


class Car:
    def __init__(self, x, y, b, img, name, piecewise_curvature, piecewise_angle, piecewise_position, parameters):
        self.fi = img
        self.name = name
        self.img = None

        # Internal State
        self.x = x
        self.y = y
        self.s = 0  # linear distance along track (meters) - GUIDE PIN position
        self.b = b  # car heading angle in radians
        self.v = 0  # linear velocity (m/s)

        # Lateral dynamics for rear slip
        self.lateral_offset_rear = 0  # how far rear is offset from centerline (meters, perpendicular to track)
        self.lateral_velocity_rear = 0  # lateral velocity of rear (m/s)

        # Inputs
        self.w = 0  # wheel angle
        self.iv = parameters["voltage"]  # input voltage

        # Parameters
        self.magnet_go = parameters["max_energy"]
        self.mass = parameters["mass"]  # in GRAMS (slot car scale)
        self.static_f = parameters["static_f"]
        self.dynamic_f = parameters["dynamic_f"]
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.back_emf = parameters["back_emf"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]
        self.R = 0.5  # internal motor resistance (ohms)
        
        # Car geometry (slot car scale in meters)
        self.wheelbase = 0.08  # 8 cm from guide pin to rear axle
        self.cg_position = 0.04  # CG is 4 cm behind guide pin (middle of car)

        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

        self.driving_state: DRIVING_STATE = "driving"
        self.animation_direction = None

    def calculate_motor_force(self, voltage, velocity):
        """
        Calculate motor force with gearbox from slide 12
        F(V, v) = (η * N * k_t / (r * R)) * (V - (k_e * N * v) / r)

        Args:
            voltage: Applied voltage (V)
            velocity: Linear velocity (m/s)

        Returns:
            Motor force in Newtons
        """
        # Slot car units - keep as-is from sliders
        r = self.wheel_r / 1000  # wheel radius: mm to meters (still need this for formula)
        eta = self.gear_efficiency / 100  # efficiency: percentage to decimal
        N = self.gear_ratio

        # Scale torque constants to realistic values for slot car
        k_t = self.torque_c * 0.01  # Scale down
        k_e = self.back_emf_c * 0.01  # Scale down

        R = self.R

        # Calculate back EMF term
        back_emf_term = (k_e * N * velocity) / r

        # Calculate motor force
        force = (eta * N * k_t / (r * R)) * (voltage - back_emf_term)

        return force

    def calculate_rear_dynamics(self):
        # === POSITION CALCULATION WITH GUIDE PIN MODEL ===

        # 1. Guide pin position (FIXED to track centerline)
        guide_x, guide_y = self.piecewise_position.get(self.s)
        track_angle = self.piecewise_angle.get(self.s)

        # 2. Rear wheel position (can be offset laterally)
        # Rear is wheelbase distance BEHIND guide pin along track direction
        # Track angle points FORWARD, so rear is in OPPOSITE direction
        rear_x_centerline = guide_x - self.wheelbase * math.cos(track_angle)
        rear_y_centerline = guide_y - self.wheelbase * math.sin(track_angle)

        # Add lateral offset (perpendicular to track)
        perpendicular_angle = track_angle + math.pi / 2  # 90° to track
        rear_x = rear_x_centerline + self.lateral_offset_rear * math.cos(perpendicular_angle)
        rear_y = rear_y_centerline + self.lateral_offset_rear * math.sin(perpendicular_angle)

        # 3. Center of Gravity position (between guide pin and rear)
        # CG is at cg_position distance behind guide pin
        cg_fraction = self.cg_position / self.wheelbase  # fraction along wheelbase
        new_x = guide_x * (1 - cg_fraction) + rear_x * cg_fraction
        new_y = guide_y * (1 - cg_fraction) + rear_y * cg_fraction

        # 4. Car heading angle (direction car is pointing)
        # Car nose points FROM rear TO guide (forward direction)
        dx, dy = guide_x - rear_x, guide_y - rear_y  # reversed: guide - rear instead of rear - guide
        new_b = math.atan2(dy, dx)

        # For now: lateral_offset_rear stays 0 (no slip yet)
        # We'll add slip dynamics late
        return new_x, new_y, new_b

    def tick(self, deltat):
        """
        Basic physics simulation

        Steps:
        1. Calculate motor force based on voltage and velocity
        2. Calculate acceleration: a = F / m
        3. Update velocity: v = v + a * dt
        4. Update position: s = s + v * dt
        5. Get x, y, angle from piecewise functions
        """
        if self.driving_state == "driving":
            # Calculate motor force (in Newtons)
            motor_force = self.calculate_motor_force(self.iv, self.v)

            # Calculate acceleration (F = m*a, so a = F/m), Force in N, mass in g → a in m/s² * 1000
            acceleration = (motor_force / self.mass) * 1000  # convert g to kg in formula

            # Update velocity using Euler integration v(t+dt) = v(t) + a * dt
            self.v += acceleration * deltat

            # Prevent negative velocity
            if self.v < 0:
                self.v = 0

            # Update position along track s(t+dt) = s(t) + v * dt
            distance_increment = self.v * deltat  # meters
            self.s += distance_increment

            self.x, self.y, self.b = self.calculate_rear_dynamics()

            if self.iv > 4:
                self.driving_state = "derailed"
                self.animation_direction = self.b * 180 / math.pi
                self.v = self.v if self.v != 0 else 0.4

        elif self.driving_state == "derailed":
            self.x = self.x + self.v * math.cos(self.animation_direction) * deltat
            self.y = self.y + self.v * math.sin(self.animation_direction) * deltat
            self.b = self.b if self.b < 6.28 else self.b - 6.28
            self.b += 0.15

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
