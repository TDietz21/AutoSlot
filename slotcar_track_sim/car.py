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

        # Internal State
        self.x = x
        self.y = y
        self.s = 0  # linear distance in mm
        self.b = b  # angle in radians
        self.v = 0  # linear velocity

        # Inputs
        self.w = 0  # wheel angle
        self.iv = parameters["voltage"]  # input voltage

        # Parameters
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
        self.R = 0.5  # internal motor resistance (ohms)
        self.axisdistance = 0.7  # 7 cm
        self.guide_torque_max = 10  # Nm (tunable)

        self.piecewise_curvature = piecewise_curvature
        self.piecewise_position = piecewise_position
        self.piecewise_angle = piecewise_angle

    def tick(self, deltat):
        # @TODO dirty hack to demo
        self.v = self.iv

        incs = self.v * deltat
        self.s += incs
        c = self.piecewise_curvature.get(self.s)  # curvature in radians/mm

        self.x, self.y = self.piecewise_position.get(self.s)

        # DEMO WRONG WAY
        # self.b += c * incs

        self.b = self.piecewise_angle.get(self.s)

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

        # print('draw car', self.x, self.y, screen_x, screen_y)

    def updateParameters(self, parameters: dict) -> None:
        self.iv = parameters["voltage"]
        self.magnet_go = parameters["max_energy"]
        self.mass = parameters["mass"]
        self.static_f = parameters["statif_f"]
        self.dynamic_f = parameters["dynamic_f"]
        self.wheel_r = parameters["wheel_r"]
        self.torque_c = parameters["torque_c"]
        self.back_emf_c = parameters["back_emf_c"]
        self.back_emf = parameters["back_emf"]
        self.gear_ratio = parameters["gear_ratio"]
        self.gear_efficiency = parameters["efficiency"]
