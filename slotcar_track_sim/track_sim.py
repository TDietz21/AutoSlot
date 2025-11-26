import tkinter as tk
import threading
import time
import math
from PIL import Image, ImageTk
from track import *
from config import *
from car2 import *
from tkinter import ttk

piecewise_function_xy1 = None


class App:
    param_definitions = [
        ("Voltage", 0.0, 12.0, 0.1, "V", "voltage"),
        ("Magnet Max Energy Product", 0.0, 50.0, 0.5, "MGOe", "max_energy"),
        ("Mass", 70.0, 200.0, 1.0, "g", "mass"),
        ("Static Friction", 0.0, 2.0, 0.01, "-", "static_f"),
        ("Dynamic Friction", 0.0, 2.0, 0.01, "-", "dynamic_f"),
        ("Wheel Radius", 4.0, 10.0, 0.1, "mm", "wheel_r"),
        ("Torque Constant", 0.8, 2.0, 0.01, "-", "torque_c"),
        ("Back EMF Constant", 1.0, 5.0, 0.01, "-", "back_emf_c"),
        ("Back EMF", 0.003, 0.007, 0.0001, "-", "back_emf"),
        ("Gear Ratio", 2.5, 4.0, 0.1, "-", "gear_ratio"),
        ("Geartrain Efficiency", 80.0, 95.0, 1.0, "%", "efficiency"),
    ]

    def __init__(self, parent):
        self.parent = parent
        self.parent.title("Simulation")
        self.parent.geometry(f"{sw}x{sh}")

        self.parameters = {}
        self.cars = []

        self.parent.grid_columnconfigure(0, weight=3)
        self.parent.grid_columnconfigure(1, weight=7)
        self.parent.grid_rowconfigure(0, weight=1)

        self.setup_control_panel()

        self.canvas = tk.Canvas(parent, bg="white")
        self.canvas.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        self.worker = threading.Thread(target=self.simulator_thread, daemon=True)
        self.worker.start()

        self.last_redraw_time = time.time()
        self.parent.after(1, self.redraw)

    def setup_control_panel(self):
        self.control_frame = ttk.Frame(self.parent, padding="10 10 10 10", relief=tk.RAISED)
        self.control_frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(self.control_frame, text="System Parameters", font=("Arial", 16, "bold")).grid(
            row=0, column=0, columnspan=3, pady=(0, 15), sticky="w"
        )

        self.sliders_container = ttk.Frame(self.control_frame)
        self.sliders_container.grid(row=1, column=0, columnspan=3, sticky="nsew")

        self.create_sliders(self.sliders_container)

        # --------------------------------------
        # 🚀 RESET BUTTON (integrated)
        # --------------------------------------
        reset_btn = ttk.Button(self.control_frame, text="Reset Simulation", command=self.reset_simulation)
        reset_btn.grid(row=2, column=0, pady=15, sticky="ew")

    def create_sliders(self, parent):
        parent.grid_columnconfigure(0, weight=1)
        parent.grid_columnconfigure(1, weight=3)
        parent.grid_columnconfigure(2, weight=1)

        row_index = 0
        for label_text, min_val, max_val, resolution, unit, var_name in self.param_definitions:
            ttk.Label(parent, text=f"{label_text}:").grid(row=row_index, column=0, padx=5, pady=5, sticky="w")

            initial_value = min_val
            self.parameters[var_name] = initial_value
            value_label = ttk.Label(parent, text=f"{initial_value:.4f} {unit}", width=12)
            value_label.grid(row=row_index, column=2, padx=5, pady=5, sticky="e")

            def update_value_wrapper(name, unit, label):
                return lambda val: self.update_value(name, unit, label, val)

            slider = ttk.Scale(
                parent,
                from_=min_val,
                to=max_val,
                orient=tk.HORIZONTAL,
                command=update_value_wrapper(var_name, unit, value_label),
            )
            slider.set(initial_value)
            slider.grid(row=row_index, column=1, padx=5, pady=5, sticky="ew")

            row_index += 1

    def update_value(self, var_name, unit, value_label, value):
        new_value = float(value)
        self.parameters[var_name] = new_value

        if var_name == "back_emf":
            formatted_value = f"{new_value:.4f}"
        elif new_value == round(new_value):
            formatted_value = f"{int(new_value)}"
        else:
            formatted_value = f"{new_value:.2f}"

        value_label.config(text=f"{formatted_value} {unit}")

        if len(self.cars) > 0:
            self.cars[0].updateParameters(self.parameters)

    # ---------------------------------------------------
    # 🚀 FULL RESET OF SIMULATION (sliders + canvas + car)
    # ---------------------------------------------------
    def reset_simulation(self):
        """Resets all sliders, parameters, cars, and the canvas."""
        # Reset slider values
        for child in self.sliders_container.winfo_children():
            if isinstance(child, ttk.Scale):
                child.set(float(child.cget("from")))

        # Reset parameters dictionary
        for _, min_val, _, _, _, var_name in self.param_definitions:
            self.parameters[var_name] = min_val

        # Clear cars
        self.cars.clear()

        # Clear canvas
        self.canvas.delete("all")

        # Rebuild circuit
        self.initCircuit()

    def initCircuit(self):
        global piecewise_function_xy1

        piecewise_function_t1 = CurvaturePiecewiseFunction()
        piecewise_function_xy1 = PositionPiecewiseFunction()
        piecewise_function_a = AnglePiecewiseFunction()

        lane_idx = 0
        initial_x, initial_y = -100 / 1000, -350 / 1000

        if lane_idx == 0:
            lane_y = LANE_SPACING / 2 + LANE_SPACING
        elif lane_idx == 1:
            lane_y = LANE_SPACING / 2

        drawTarmac = True
        drawParametricCurve = True

        x, y, a = initial_x, initial_y, 0

        t = C8205Track(x, y, a)
        if drawTarmac:
            t.draw(self.canvas)
        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8205Track(x, y, a)
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        t = C8204Track(x, y, a, "L")
        if drawTarmac:
            t.draw(self.canvas)

        piecewise_function_t1.appendTrack(t, lane_idx)
        piecewise_function_xy1.appendTrack(t, lane_idx)
        piecewise_function_a.appendTrack(t, lane_idx)
        x, y, a = t.getNext()

        s = np.linspace(0, piecewise_function_t1.getLength(), 1000)[0:-1]

        coords_list = [piecewise_function_xy1.get(s_i) for s_i in s]
        coords_list = [m_to_px(self.canvas, x, y) for x, y in coords_list]

        if drawParametricCurve:
            self.canvas.create_line(*coords_list, fill="darkorange", width=10, smooth=True)

        self.cars.append(
            Car(
                initial_x,
                (initial_y + lane_y),
                0,
                car1_img,
                "car 1",
                piecewise_function_t1,
                piecewise_function_a,
                piecewise_function_xy1,
                self.parameters,
            )
        )

    def redraw(self):
        current_time = time.time()
        if current_time - self.last_redraw_time >= deltat:
            for car in self.cars:
                car.tick(deltat)
                car.draw(self.canvas)
            self.last_redraw_time = current_time

        self.parent.after(1, self.redraw)

    def simulator_thread(self):
        time.sleep(0.5)
        self.parent.after(0, self.initCircuit)
        while True:
            time.sleep(0.1)


if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()
