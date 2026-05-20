import contextlib
import json
import queue
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import circularmove
import config
from Dobot import disconnect_robot, get_robot_error, turn_do_off


SETTINGS_PATH = Path(__file__).with_name("circularmove_ui_settings.json")


class CircularMoveGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dobot Circular Move")
        self.geometry("800x760")

        self.laser = None
        self.dobot = None
        self.feed_thread = None
        self.saved_start_pose = None
        self.initial_pose = None
        self.initialized = False
        self.running = False
        self.worker = None
        self.loop_wavelengths = []

        self.log_queue = queue.Queue()
        self.done_token = object()
        self.stop_event = threading.Event()
        self.return_event = threading.Event()

        self.inputs = {}
        self.settings = self._load_settings()
        self.loop_count_var = tk.StringVar(value=str(config.LOOP_REPEAT_COUNT))
        self.wavelength_entries = []
        saved_tool_frame = self.settings.get("TOOL_FRAME", config.TOOL_FRAME)
        self.current_tool_index_var = tk.StringVar(value="Not initialized")
        self.current_tool_frame_var = tk.StringVar(value=f"Saved value: {saved_tool_frame}")

        self.build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.drain_log)

    def build_ui(self):
        notebook = ttk.Notebook(self)
        notebook.pack(fill="x", padx=12, pady=10)

        move_page = ttk.Frame(notebook, padding=10)
        self.tool_page = ttk.Frame(notebook, padding=10)
        notebook.add(move_page, text="Circular Move")
        notebook.add(self.tool_page, text="Tool Frame")

        form = ttk.LabelFrame(move_page, text="Circular Move Parameters", padding=12)
        form.pack(fill="x")

        fields = [
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Circle Radius mm", "CIRCLE_RADIUS_MM", config.CIRCLE_RADIUS_MM),
            ("Circle End Angle deg", "CIRCLE_END_DEG", config.CIRCLE_END_DEG),
            ("Circle Total Steps", "CIRCLE_TOTAL_STEPS", config.CIRCLE_TOTAL_STEPS),
            ("Trigger DO Index", "TRIGGER_DO_INDEX", config.TRIGGER_DO_INDEX),
            ("Trigger Pulse Seconds", "TRIGGER_PULSE_SECONDS", config.TRIGGER_PULSE_SECONDS),
            ("Loop Repeat Count", "LOOP_REPEAT_COUNT", config.LOOP_REPEAT_COUNT),
            ("Motion User Index", "CIRCLE_USER_INDEX", config.CIRCLE_USER_INDEX),
            ("Motion Tool Index", "CIRCLE_TOOL_INDEX", config.CIRCLE_TOOL_INDEX),
            ("Acceleration Ratio", "CIRCLE_ACCELERATION_RATIO", config.CIRCLE_ACCELERATION_RATIO),
            ("Velocity Ratio", "CIRCLE_VELOCITY_RATIO", config.CIRCLE_VELOCITY_RATIO),
            ("CP", "CIRCLE_CP", config.CIRCLE_CP),
            ("Circle Rx deg", "CIRCLE_RX_DEG", config.CIRCLE_RX_DEG),
            ("Circle Start Ry deg", "CIRCLE_START_RY_DEG", config.CIRCLE_START_RY_DEG),
            ("Circle Rz deg", "CIRCLE_RZ_DEG", config.CIRCLE_RZ_DEG),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(form, width=38)
            saved = self.settings.get(key, value)
            if key == "LOOP_REPEAT_COUNT":
                self.loop_count_var.set(str(saved))
                entry.configure(textvariable=self.loop_count_var)
            else:
                entry.insert(0, str(saved))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        form.columnconfigure(1, weight=1)

        self.wavelength_frame = ttk.LabelFrame(move_page, text="Loop Wavelengths", padding=12)
        self.wavelength_frame.pack(fill="x", pady=(10, 0))
        self.update_wavelength_entries()
        self.loop_count_var.trace_add(
            "write", lambda *_: self.after_idle(self.update_wavelength_entries)
        )

        self.build_tool_frame_page()

        buttons = ttk.Frame(self, padding=(12, 0))
        buttons.pack(fill="x")
        self.init_button = ttk.Button(buttons, text="Initialize", command=self.initialize_devices)
        self.start_button = ttk.Button(buttons, text="Start", command=self.start_experiment, state="disabled")
        self.stop_button = ttk.Button(buttons, text="Stop", command=self.stop_experiment, state="disabled")
        self.return_button = ttk.Button(buttons, text="Return To Start", command=self.return_to_start, state="disabled")

        self.init_button.pack(side="left", padx=(0, 8))
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button.pack(side="left", padx=(0, 8))
        self.return_button.pack(side="left")

        log_frame = ttk.LabelFrame(self, text="Log", padding=8)
        log_frame.pack(fill="both", expand=True, padx=12, pady=10)
        self.log_text = tk.Text(log_frame, height=14, wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def build_tool_frame_page(self):
        current_frame = ttk.LabelFrame(self.tool_page, text="Current Tool Frame", padding=12)
        current_frame.pack(fill="x")

        ttk.Label(current_frame, text="Active Tool Index").grid(row=0, column=0, sticky="w", pady=4)
        ttk.Label(current_frame, textvariable=self.current_tool_index_var).grid(
            row=0, column=1, sticky="w", pady=4
        )
        ttk.Label(current_frame, text="Active Tool Frame").grid(row=1, column=0, sticky="nw", pady=4)
        ttk.Label(
            current_frame,
            textvariable=self.current_tool_frame_var,
            wraplength=620,
        ).grid(row=1, column=1, sticky="ew", pady=4)
        current_frame.columnconfigure(1, weight=1)

        edit_frame = ttk.LabelFrame(self.tool_page, text="Tool Frame Settings", padding=12)
        edit_frame.pack(fill="x", pady=(10, 0))

        fields = [
            ("Tool Index", "TOOL_INDEX", config.TOOL_INDEX),
            ("Tool Frame", "TOOL_FRAME", config.TOOL_FRAME),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(edit_frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(edit_frame, width=38)
            entry.insert(0, str(self.settings.get(key, value)))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        edit_frame.columnconfigure(1, weight=1)

        tool_buttons = ttk.Frame(self.tool_page)
        tool_buttons.pack(fill="x", pady=(10, 0))
        self.activate_tool_button = ttk.Button(
            tool_buttons,
            text="Activate Tool Frame",
            command=self.activate_tool_frame,
            state="disabled",
        )
        self.refresh_tool_button = ttk.Button(
            tool_buttons,
            text="Refresh Current",
            command=self.refresh_tool_frame_display,
            state="disabled",
        )
        self.activate_tool_button.pack(side="left", padx=(0, 8))
        self.refresh_tool_button.pack(side="left")

    def _load_settings(self):
        try:
            with SETTINGS_PATH.open("r", encoding="utf-8") as file:
                return json.load(file)
        except Exception:
            return {}

    def _save_settings(self):
        settings = {key: entry.get() for key, entry in self.inputs.items()}
        settings["WAVELENGTHS"] = [entry.get() for entry in self.wavelength_entries]
        try:
            with SETTINGS_PATH.open("w", encoding="utf-8") as file:
                json.dump(settings, file, indent=2)
        except Exception:
            pass

    def read_tool_parameters(self, apply=True):
        tool_index = int(float(self.inputs["TOOL_INDEX"].get()))
        tool_frame = self.inputs["TOOL_FRAME"].get().strip()

        if tool_index < 0:
            raise ValueError("Tool Index cannot be negative")
        if not tool_frame:
            raise ValueError("Tool Frame cannot be empty")

        if apply:
            config.TOOL_INDEX = tool_index
            config.TOOL_FRAME = tool_frame
        return tool_index, tool_frame

    def format_tool_frame(self, values):
        formatted_values = []
        for value in values:
            text = f"{float(value):.6f}".rstrip("0").rstrip(".")
            formatted_values.append(text or "0")
        return "{" + ",".join(formatted_values) + "}"

    def refresh_tool_frame_display(self):
        if self.dobot is None or not self.initialized:
            self.current_tool_index_var.set("Not initialized")
            self.current_tool_frame_var.set(
                f"Saved value: {self.inputs['TOOL_FRAME'].get().strip()}"
            )
            return

        try:
            tool_index, tool_frame = self.dobot.GetCurrentToolFrame()
        except Exception as error:
            self.current_tool_index_var.set("Unavailable")
            self.current_tool_frame_var.set(str(error))
            return

        self.current_tool_index_var.set(str(tool_index))
        self.current_tool_frame_var.set(self.format_tool_frame(tool_frame))

    def activate_tool_frame(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.initialized or self.dobot is None:
            messagebox.showerror("Not Initialized", "Initialize robot before activating a tool frame.")
            return

        try:
            self.read_tool_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Tool Frame", str(error))
            return

        self._save_settings()
        self.log("Activating tool frame")
        self.running = False
        self.set_busy(True)

        def worker():
            try:
                with contextlib.redirect_stdout(self):
                    set_tool_result = self.dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
                    activate_result = self.dobot.ActivateTool(config.TOOL_INDEX)
                    print("SetTool result:", set_tool_result)
                    print("ActivateTool result:", activate_result)
                    time.sleep(0.2)
                self.log("Tool frame activated")
            except Exception as error:
                self.log(f"ERROR: {error}")
            finally:
                self.log_queue.put(self.done_token)

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def update_wavelength_entries(self):
        try:
            loop_count = int(float(self.loop_count_var.get()))
        except (ValueError, tk.TclError):
            return
        if loop_count <= 0 or loop_count > 100:
            return

        old_values = [entry.get() for entry in self.wavelength_entries]
        saved_wavelengths = self.settings.get("WAVELENGTHS", [])
        for child in self.wavelength_frame.winfo_children():
            child.destroy()
        self.wavelength_entries = []

        for index in range(loop_count):
            if index < len(old_values):
                value = old_values[index]
            elif index < len(saved_wavelengths):
                value = saved_wavelengths[index]
            else:
                value = str(config.LASER_WAVELENGTH_NM)

            ttk.Label(self.wavelength_frame, text=f"Loop {index + 1} Wavelength nm").grid(
                row=index, column=0, sticky="w", pady=3
            )
            entry = ttk.Entry(self.wavelength_frame, width=38)
            entry.insert(0, value)
            entry.grid(row=index, column=1, sticky="ew", pady=3)
            self.wavelength_entries.append(entry)

        self.wavelength_frame.columnconfigure(1, weight=1)

    def read_parameters(self):
        speed_ratio = int(float(self.inputs["SPEED_RATIO"].get()))
        radius = float(self.inputs["CIRCLE_RADIUS_MM"].get())
        end_angle = float(self.inputs["CIRCLE_END_DEG"].get())
        total_steps = int(float(self.inputs["CIRCLE_TOTAL_STEPS"].get()))
        do_index = int(float(self.inputs["TRIGGER_DO_INDEX"].get()))
        pulse_seconds = float(self.inputs["TRIGGER_PULSE_SECONDS"].get())
        loop_count = int(float(self.loop_count_var.get()))
        user_index = int(float(self.inputs["CIRCLE_USER_INDEX"].get()))
        motion_tool_index = int(float(self.inputs["CIRCLE_TOOL_INDEX"].get()))
        acceleration = int(float(self.inputs["CIRCLE_ACCELERATION_RATIO"].get()))
        velocity = int(float(self.inputs["CIRCLE_VELOCITY_RATIO"].get()))
        cp = int(float(self.inputs["CIRCLE_CP"].get()))
        rx = float(self.inputs["CIRCLE_RX_DEG"].get())
        start_ry = float(self.inputs["CIRCLE_START_RY_DEG"].get())
        rz = float(self.inputs["CIRCLE_RZ_DEG"].get())
        wavelengths = [float(entry.get().strip()) for entry in self.wavelength_entries]
        tool_index, tool_frame = self.read_tool_parameters(apply=False)

        if speed_ratio <= 0:
            raise ValueError("Speed Ratio must be greater than 0")
        if radius <= 0:
            raise ValueError("Circle Radius mm must be greater than 0")
        if end_angle <= 0:
            raise ValueError("Circle End Angle deg must be greater than 0")
        if total_steps <= 0:
            raise ValueError("Circle Total Steps must be greater than 0")
        if do_index <= 0:
            raise ValueError("Trigger DO Index must be greater than 0")
        if pulse_seconds <= 0:
            raise ValueError("Trigger Pulse Seconds must be greater than 0")
        if loop_count <= 0:
            raise ValueError("Loop Repeat Count must be greater than 0")
        if len(wavelengths) != loop_count:
            raise ValueError("Loop wavelength count must match Loop Repeat Count")
        if any(wavelength <= 0 for wavelength in wavelengths):
            raise ValueError("Each wavelength must be greater than 0")
        if user_index < 0 or motion_tool_index < 0:
            raise ValueError("User and motion tool indexes cannot be negative")
        if acceleration <= 0 or velocity <= 0:
            raise ValueError("Acceleration and Velocity must be greater than 0")
        if cp < 0:
            raise ValueError("CP cannot be negative")

        config.SPEED_RATIO = speed_ratio
        config.CIRCLE_RADIUS_MM = radius
        config.CIRCLE_END_DEG = end_angle
        config.CIRCLE_TOTAL_STEPS = total_steps
        config.TRIGGER_DO_INDEX = do_index
        config.TRIGGER_PULSE_SECONDS = pulse_seconds
        config.LOOP_REPEAT_COUNT = loop_count
        config.CIRCLE_USER_INDEX = user_index
        config.CIRCLE_TOOL_INDEX = motion_tool_index
        config.CIRCLE_ACCELERATION_RATIO = acceleration
        config.CIRCLE_VELOCITY_RATIO = velocity
        config.CIRCLE_CP = cp
        config.CIRCLE_RX_DEG = rx
        config.CIRCLE_START_RY_DEG = start_ry
        config.CIRCLE_RZ_DEG = rz
        config.TOOL_INDEX = tool_index
        config.TOOL_FRAME = tool_frame
        config.LASER_WAVELENGTH_NM = wavelengths[0]
        self.loop_wavelengths = wavelengths
        self._save_settings()

    def initialize_devices(self):
        if self.worker and self.worker.is_alive():
            return
        try:
            self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.log("Initializing robot and laser")
        self.running = False
        self.set_busy(True)

        def worker():
            try:
                with contextlib.redirect_stdout(self):
                    (
                        self.laser,
                        self.dobot,
                        self.feed_thread,
                        self.saved_start_pose,
                        self.initial_pose,
                    ) = circularmove.initialize()
                self.initialized = True
                self.log("Initialization complete")
            except Exception as error:
                self.log(f"ERROR: {error}")
            finally:
                self.log_queue.put(self.done_token)

        self.worker = threading.Thread(target=worker, daemon=True)
        self.worker.start()

    def start_experiment(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.initialized:
            messagebox.showerror("Not Initialized", "Initialize robot and laser before starting.")
            return
        try:
            self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.stop_event.clear()
        self.return_event.clear()
        self.running = True
        self.set_busy(True)
        self.log("Starting circular move")

        self.worker = threading.Thread(target=self.run_experiment, daemon=True)
        self.worker.start()

    def run_experiment(self):
        try:
            with contextlib.redirect_stdout(self):
                angle_step_deg = config.CIRCLE_END_DEG / config.CIRCLE_TOTAL_STEPS
                poses = circularmove.generate_xz_circle_poses(
                    self.initial_pose,
                    radius=config.CIRCLE_RADIUS_MM,
                    angle_step_deg=angle_step_deg,
                    end_angle_deg=config.CIRCLE_END_DEG,
                    rx=config.CIRCLE_RX_DEG,
                    start_ry=config.CIRCLE_START_RY_DEG,
                    rz=config.CIRCLE_RZ_DEG,
                )
                print("Circle point count:", len(poses))

                if get_robot_error(self.dobot):
                    self.return_to_saved_start()
                    return

                set_tool_result = self.dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
                activate_result = self.dobot.ActivateTool(config.TOOL_INDEX)
                print("SetTool result:", set_tool_result)
                print("ActivateTool result:", activate_result)

                self.laser.run()
                print("Laser RUN")
                if self.wait_seconds(5):
                    return

                for loop_index, wavelength in enumerate(self.loop_wavelengths, start=1):
                    if self.stop_or_return_requested():
                        return
                    if get_robot_error(self.dobot):
                        self.return_to_saved_start()
                        return

                    print(f"Loop {loop_index}/{len(self.loop_wavelengths)}")
                    self.laser.set_wavelength(wavelength)
                    if self.wait_seconds(2):
                        return

                    for point_index, pose in enumerate(poses, start=1):
                        if self.stop_or_return_requested():
                            return
                        if get_robot_error(self.dobot):
                            self.return_to_saved_start()
                            return

                        print(f"Circle point {point_index}/{len(poses)}:", pose)
                        circularmove.run_step(
                            self.dobot,
                            pose,
                            user=config.CIRCLE_USER_INDEX,
                            tool=config.CIRCLE_TOOL_INDEX,
                            acceleration=config.CIRCLE_ACCELERATION_RATIO,
                            velocity=config.CIRCLE_VELOCITY_RATIO,
                            cp=config.CIRCLE_CP,
                        )

                    if get_robot_error(self.dobot):
                        self.return_to_saved_start()
                        return

                    circularmove.run_step(
                        self.dobot,
                        self.initial_pose,
                        user=config.CIRCLE_USER_INDEX,
                        tool=config.CIRCLE_TOOL_INDEX,
                        acceleration=config.CIRCLE_ACCELERATION_RATIO,
                        velocity=config.CIRCLE_VELOCITY_RATIO,
                        cp=config.CIRCLE_CP,
                    )
                    if self.wait_seconds(2):
                        return
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            if self.laser is not None:
                self.laser.stop_safely()
            self.log("Circular move finished")
            self.log_queue.put(self.done_token)

    def wait_seconds(self, seconds):
        end_time = time.perf_counter() + seconds
        while True:
            if self.stop_or_return_requested():
                return True
            remaining = end_time - time.perf_counter()
            if remaining <= 0:
                return False
            time.sleep(min(0.1, remaining))

    def stop_or_return_requested(self):
        if self.stop_event.is_set() or self.return_event.is_set():
            self.return_to_saved_start()
            return True
        return False

    def return_to_saved_start(self):
        if self.laser is not None:
            self.laser.stop_safely()
        if self.dobot is None or self.saved_start_pose is None:
            return
        turn_do_off(self.dobot, config.TRIGGER_DO_INDEX)
        self.dobot.MoveLinearPoint(self.saved_start_pose, config.SPEED_RATIO)

    def stop_experiment(self):
        self.log("Stop requested")
        self.stop_event.set()

    def return_to_start(self):
        self.log("Return to start requested")
        self.return_event.set()

    def set_busy(self, busy):
        self.init_button.configure(state="disabled" if busy else "normal")
        self.start_button.configure(state="disabled" if busy or not self.initialized else "normal")
        self.stop_button.configure(state="normal" if self.running else "disabled")
        self.return_button.configure(state="normal" if self.running else "disabled")
        tool_state = "normal" if self.initialized and not busy else "disabled"
        self.activate_tool_button.configure(state=tool_state)
        self.refresh_tool_button.configure(state=tool_state)

    def log(self, text):
        self.log_queue.put(text)

    def write(self, text):
        if text.strip():
            self.log(text)

    def flush(self):
        pass

    def drain_log(self):
        while True:
            try:
                text = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if text is self.done_token:
                self.running = False
                self.set_busy(False)
                self.refresh_tool_frame_display()
            else:
                self.log_text.insert("end", text.rstrip() + "\n")
                self.log_text.see("end")
        self.after(100, self.drain_log)

    def on_close(self):
        self.stop_event.set()
        self.return_event.set()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=120)
        if self.laser is not None:
            try:
                self.laser.close()
            except Exception:
                pass
        if self.dobot is not None:
            try:
                disconnect_robot(self.dobot)
            except Exception:
                pass
        self.destroy()


def start_gui():
    app = CircularMoveGui()
    app.mainloop()


def main():
    start_gui()


if __name__ == "__main__":
    main()
