# -*- coding: utf-8 -*-
"""
Created on Fri Oct 17 09:06:36 2025

@author: 2016570
"""
import math
import numpy as np
from config import *

LANE_SPACING = 78/1000        # center-to-center between lanes 
RAIL_SPACING = 10/1000         # rail center-to-center distance
SLOT_WIDTH = 3/1000           # slot width
RAIL_WIDTH = 5/1000
TRACK_WIDTH = 155/1000        # total track piece width

R1_RADIUS = 214/1000
R2_RADIUS = 370/1000
R3_RADIUS = 526/1000
R4_RADIUS = 682/1000



class Track:
    def getLaneStart(self, lane_idx):
        if (lane_idx  == 0):
            lane_y = (LANE_SPACING/2+LANE_SPACING) 
        elif (lane_idx == 1):
            lane_y = (LANE_SPACING/2) 
            
        angle = self.angle + math.pi/2
        x = self.x + lane_y * math.cos(angle)
        y = self.y + lane_y * math.sin(angle)
        
        return x,y


class CurvaturePiecewiseFunction:
    def __init__(self):
        self.piece = [] # curvature , start, end
        
        
    def appendTrack(self, o, t):
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
            
        c = o.getLaneCurvature(t)
        end = start + o.getLaneLength(t)
        
        self.piece.append((c, start, end))

    def getLength(self):
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
        return start
    
    def get(self, x):
        l = self.getLength()
        
        if (x > l):
            x -= int(x/l) * l
            
        for c, s, e in self.piece:
            if (x >= s) and (x < e):
                return c
            
        raise Exception('x = ', x)

class AnglePiecewiseFunction:
    def __init__(self):
        self.piece = [] # curvature , start, end
        
        
    def appendTrack(self, o, t):
        a0 = o.angle
        
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
            
        c = o.getLaneCurvature(t)
        end = start + o.getLaneLength(t)
        
        self.piece.append((c, start, end, a0))

    def getLength(self):
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
        return start
    
    def get(self, x):
        l = self.getLength()
        
        if (x > l):
            x -= int(x/l) * l
            
        for c, s, e, a0 in self.piece:
            if (x >= s) and (x < e):
                rx = x - s
                af = a0 + rx * c
                return af
            
        raise Exception('x = ', x)        
        
class PositionPiecewiseFunction:
    def __init__(self):
        self.piece = [] # curvature , parametric start, parametric end, x start, y start, initial angle
        
    def appendTrack(self, o, t):
        x0, y0 = o.getLaneStart(t)
        a0 = o.angle
        
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
            
        c = o.getLaneCurvature(t)
        end = start + o.getLaneLength(t)
        
        self.piece.append((c, start, end, x0, y0, a0))

    def getLength(self):
        if (len(self.piece) == 0):
            start = 0
        else:
            start = self.piece[-1][2] # previous end
        return start
    
    def get(self, x):
        l = self.getLength()
        
        if (x > l):
            x -= int(x/l) * l
            
        for c, s, e, x0, y0, a0 in self.piece:
            if (x >= s) and (x < e):
                rx = x - s
                
                # compute the x,y position for the parameter
                if (c == 0):
                    xr = x0 + rx * math.cos(a0)
                    yr = y0 + rx * math.sin(a0)
                    
                    return xr , yr 
                else:
                    af = a0 + rx * c
                    xr = x0 + 1/c * (math.sin(af)-math.sin(a0))
                    yr = y0 - 1/c * (math.cos(af)-math.cos(a0))
                    return xr, yr
            
        raise Exception('x = ', x, 'l=', l)        
        
class StraighTrack(Track):
    
    def getNext(self):
        # returns the connection point and angle of the next piece
        # in world coordinates (meters) and degree angles
        rads = self.angle 
        nx = (self.x + self.TRACK_LENGTH * math.cos(rads)) 
        ny = (self.y + self.TRACK_LENGTH * math.sin(rads)) 
        return nx, ny, self.angle
        
    def getLaneCurvature(self, t):
        return 0
    
    def getLaneLength(self, t):
        return self.TRACK_LENGTH
    
    def draw(self, canvas):
        lane1_center_y = (TRACK_WIDTH - LANE_SPACING) / 2
        lane2_center_y = lane1_center_y + LANE_SPACING
        self.lanes_y = [lane1_center_y, lane2_center_y]
        
        rads = self.angle 
        
        # Get the rotation center (x0, y0) in pixels
        x0, y0 = m_to_px(canvas, self.x, self.y)
        
        # Calculate the four corners of the track rectangle relative to (x0, y0)
        corners_rel = [
            (0, 0),  # top-left (rotation center)
            (self.TRACK_LENGTH, 0),  # top-right
            (self.TRACK_LENGTH, TRACK_WIDTH),  # bottom-right
            (0, TRACK_WIDTH)  # bottom-left
        ]
        
        # Rotate corners and convert to pixel coordinates
        rotated_corners = []
        for dx, dy in corners_rel:
            # Apply rotation relative to (0,0)
            rotated_dx = dx * math.cos(rads) - dy * math.sin(rads)
            rotated_dy = dx * math.sin(rads) + dy * math.cos(rads)
            # Translate to pixel coordinates (already includes x0, y0 as base)
            px = x0 + m_to_px(canvas, rotated_dx, rotated_dy)[0] - m_to_px(canvas, 0, 0)[0]
            py = y0 + m_to_px(canvas, rotated_dx, rotated_dy)[1] - m_to_px(canvas, 0, 0)[1]
            rotated_corners.extend([px, py])
        
        # Draw track background (rotated rectangle)
        canvas.create_polygon(rotated_corners, fill="gray20", outline="black", width=2)
        
        w_rails = sm_to_px(RAIL_WIDTH)
        w_slot = sm_to_px(SLOT_WIDTH)
        
        # === Draw lanes (slots + rails) ===
        for lane_y in self.lanes_y:
            # Define points along the track for slot and rails
            slot_points = []
            rail1_points = []
            rail2_points = []
            
            # Create points along the track length (start and end)
            for x_offset in [0, self.TRACK_LENGTH]:
                # Slot points
                slot_x = x_offset
                slot_y = lane_y
                # Rotate slot points relative to (0,0)
                rotated_slot_x = slot_x * math.cos(rads) - slot_y * math.sin(rads)
                rotated_slot_y = slot_x * math.sin(rads) + slot_y * math.cos(rads)
                # Convert to pixel coordinates relative to x0, y0
                px = x0 + m_to_px(canvas, rotated_slot_x, rotated_slot_y)[0] - m_to_px(canvas, 0, 0)[0]
                py = y0 + m_to_px(canvas, rotated_slot_x, rotated_slot_y)[1] - m_to_px(canvas, 0, 0)[1]
                slot_points.extend([px, py])
                
                # Rail 1 points (above slot)
                rail1_y = lane_y - RAIL_SPACING / 2
                rotated_rail1_x = x_offset * math.cos(rads) - rail1_y * math.sin(rads)
                rotated_rail1_y = x_offset * math.sin(rads) + rail1_y * math.cos(rads)
                px = x0 + m_to_px(canvas, rotated_rail1_x, rotated_rail1_y)[0] - m_to_px(canvas, 0, 0)[0]
                py = y0 + m_to_px(canvas, rotated_rail1_x, rotated_rail1_y)[1] - m_to_px(canvas, 0, 0)[1]
                rail1_points.extend([px, py])
                
                # Rail 2 points (below slot)
                rail2_y = lane_y + RAIL_SPACING / 2
                rotated_rail2_x = x_offset * math.cos(rads) - rail2_y * math.sin(rads)
                rotated_rail2_y = x_offset * math.sin(rads) + rail2_y * math.cos(rads)
                px = x0 + m_to_px(canvas, rotated_rail2_x, rotated_rail2_y)[0] - m_to_px(canvas, 0, 0)[0]
                py = y0 + m_to_px(canvas, rotated_rail2_x, rotated_rail2_y)[1] - m_to_px(canvas, 0, 0)[1]
                rail2_points.extend([px, py])
            
            # Draw rails (metallic lines)
            canvas.create_line(rail1_points[0], rail1_points[1], rail1_points[2], rail1_points[3], 
                              fill="silver", width=w_rails)
            canvas.create_line(rail2_points[0], rail2_points[1], rail2_points[2], rail2_points[3], 
                              fill="silver", width=w_rails)
            
            # Draw slot (dark groove)
            canvas.create_line(slot_points[0], slot_points[1], slot_points[2], slot_points[3], 
                              fill="black", width=w_slot)
            


    

class CurvedTrack(Track):

    def getNext(self):
        # returns the connection point and angle of the next piece
        # in world coordinates (meters)
        curve_rads = (self.angle + self.ANGLE) 
        
        cx, cy = self.getCenterOfRotation()
        
        nx = (cx + self.OUTER_RADIUS * math.sin(curve_rads)) 
        ny = (cy - self.OUTER_RADIUS * math.cos(curve_rads)) 
        return nx, ny, self.angle + self.ANGLE
    
    def getCenterOfRotation(self):
        # Get center of rotation in world coordinates (mm)
        rads = self.angle 
        
        inner_edge_radius = self.OUTER_RADIUS - TRACK_WIDTH
        outer_edge_radius = self.OUTER_RADIUS 

        # Calculate center coordinates relative to rotation center
        center_x_rel = 0  # relative to rotation center
        center_y_rel = inner_edge_radius + TRACK_WIDTH
        
        # Rotate the center point around (x0, y0)
        rotated_center_x_rel = center_x_rel * math.cos(rads) - center_y_rel * math.sin(rads)
        rotated_center_y_rel = center_x_rel * math.sin(rads) + center_y_rel * math.cos(rads)
        
        center_x = self.x + rotated_center_x_rel
        center_y = self.y + rotated_center_y_rel
        
        return center_x, center_y

    def getLaneCurvature(self, t):
        # returns curvature in radians per m
        rads = self.ANGLE 
        return rads / self.getLaneLength(t)
    
    def getLaneLength(self, t):
        rads = self.ANGLE # angle in rads
        r = self.getLaneRadius(t)
        return rads * r  
    
    def getLaneRadius(self, t):
        # --- Draw black tarmac (outer track boundary) ---
        inner_edge_radius = self.OUTER_RADIUS - TRACK_WIDTH
        outer_edge_radius = self.OUTER_RADIUS 

        lanes_radii = [inner_edge_radius + LANE_SPACING/2, self.OUTER_RADIUS - LANE_SPACING/2]
        return lanes_radii[t]
        
    def draw(self, canvas):
        rads = self.angle # angle in rads
        
        # Get the rotation center (x0, y0) in pixels
        x0, y0 = m_to_px(canvas, self.x, self.y)
        
        # center of the circle (assuming top-left origin)
        angle_start = -math.pi/2  
        angle_extent = self.ANGLE

        # --- Draw black tarmac (outer track boundary) ---
        inner_edge_radius = self.OUTER_RADIUS - TRACK_WIDTH
        outer_edge_radius = self.OUTER_RADIUS 

        # Calculate center coordinates relative to rotation center
        center_x_rel = 0  # relative to rotation center
        center_y_rel = inner_edge_radius + TRACK_WIDTH
        
        # Rotate the center point around (x0, y0)
        rotated_center_x_rel = center_x_rel * math.cos(rads) - center_y_rel * math.sin(rads)
        rotated_center_y_rel = center_x_rel * math.sin(rads) + center_y_rel * math.cos(rads)
        
        # Convert to absolute pixel coordinates
        center_x = x0 + m_to_px(canvas, rotated_center_x_rel, rotated_center_y_rel)[0] - m_to_px(canvas, 0, 0)[0]
        center_y = y0 + m_to_px(canvas, rotated_center_x_rel, rotated_center_y_rel)[1] - m_to_px(canvas, 0, 0)[1]

        lanes_radii = [inner_edge_radius + LANE_SPACING/2, self.OUTER_RADIUS - LANE_SPACING/2]

        inner_r_px = inner_edge_radius * pixels_per_meter
        outer_r_px = outer_edge_radius * pixels_per_meter

        # Calculate bounding boxes for arcs
        bbox_inner = (center_x - inner_r_px, center_y - inner_r_px,
                      center_x + inner_r_px, center_y + inner_r_px)
        bbox_outer = (center_x - outer_r_px, center_y - outer_r_px,
                      center_x + outer_r_px, center_y + outer_r_px)

        # Adjust start angle based on rotation
        rotated_start_angle = angle_start + self.angle

        # convert rads to degrees
        rotated_start_angle = rotated_start_angle * 180 / math.pi
        angle_extent = angle_extent * 180 / math.pi

        # Draw filled arc between outer and inner edge
        canvas.create_arc(bbox_outer, start=rotated_start_angle, extent=angle_extent,
                          style='pieslice', outline='', fill='gray20')
        canvas.create_arc(bbox_inner, start=rotated_start_angle-1, extent=angle_extent+1,
                          style='pieslice', outline='', fill=canvas['bg'])

        w_rails = sm_to_px(RAIL_WIDTH)
        w_slot = sm_to_px(SLOT_WIDTH)

        for radius in lanes_radii:
            # Rails relative to lane center
            rail_offset = RAIL_SPACING / 2
            rail_inner_radius = radius - rail_offset
            rail_outer_radius = radius + rail_offset

            # Convert mm to pixels
            r_inner_px = rail_inner_radius * pixels_per_meter
            r_outer_px = rail_outer_radius * pixels_per_meter

            # Draw rails as arcs
            canvas.create_arc(
                center_x - r_inner_px, center_y - r_inner_px,
                center_x + r_inner_px, center_y + r_inner_px,
                start=rotated_start_angle, extent=angle_extent,
                style='arc', outline='silver', width=w_rails
            )
            canvas.create_arc(
                center_x - r_outer_px, center_y - r_outer_px,
                center_x + r_outer_px, center_y + r_outer_px,
                start=rotated_start_angle, extent=angle_extent,
                style='arc', outline='silver', width=w_rails
            )

            # Draw slot as an arc (center line)
            canvas.create_arc(
                center_x - radius * pixels_per_meter, center_y - radius * pixels_per_meter,
                center_x + radius * pixels_per_meter, center_y + radius * pixels_per_meter,
                start=rotated_start_angle, extent=angle_extent,
                style='arc', outline='black', width=w_slot
            )

class NoTrack(StraighTrack):
    TRACK_LENGTH = 0       # standard straight length
    
    def __init__(self, x, y, angle):
        self.x , self.y = x, y
        self.angle = angle

            
class C8205Track(StraighTrack):
    TRACK_LENGTH = 350/1000       # standard straight length
    
    def __init__(self, x, y, angle):
        self.x , self.y = x, y
        self.angle = angle


class C8204Track(CurvedTrack):
    
    OUTER_RADIUS = R2_RADIUS # inner lane radius (mm)
    ANGLE = math.pi / 4               # curve angle in radians

    def __init__(self, x, y, angle, side):
        self.x , self.y = x, y
        self.angle = angle
        self.side = side

            
class C8202Track(CurvedTrack):

    OUTER_RADIUS = R1_RADIUS # inner lane radius (mm)
    ANGLE = math.pi / 4               # curve angle in radians

    def __init__(self, x, y, angle, side):
        self.x , self.y = x, y
        self.angle = angle
        self.side = side


