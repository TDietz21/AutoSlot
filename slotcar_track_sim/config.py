# -*- coding: utf-8 -*-
"""
Created on Fri Oct 17 09:07:49 2025

@author: 2016570
"""

SCALE = 0.5
deltat = 0.010  # 5 ms
gravity = 9.81  # m/s^2
pixels_per_meter = 128 * SCALE / 0.12
sw = 1920
sh = 1080
# ixels_per_mm = pixels_per_meter / 1000


# def mm_to_px(canvas, x, y):
#    """Convert mm to pixel coordinates (Tkinter uses y downward)."""
#    return (canvas.winfo_width()/2 +  (x * pixels_per_mm), canvas.winfo_height()/2 - (y * pixels_per_mm))


def m_to_px(canvas, x, y):
    """Convert mm to pixel coordinates (Tkinter uses y downward)."""
    return (canvas.winfo_width() / 2 + (x * pixels_per_meter), canvas.winfo_height() / 2 - (y * pixels_per_meter))


# def smm_to_px(s):
#    # scalar in mm to pixels
#    return s * pixels_per_mm


def sm_to_px(s):
    # scalar in mm to pixels
    return s * pixels_per_meter
