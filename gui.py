import contextlib
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

import config
import experiment1
from Dobot import calculate_step_count, disconnect_robot, get_robot_error, run_step, turn_do_off


DIRECTIONS = ("x+", "x-", "y+", "y-", "z+", "z-")


class ExperimentGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dobot Laser Experiment")
        self.geometry("760x620")

        self.laser = None
        self.dobot = None
        self.feed_thread = None
        self.saved_start_pose = None
        self.initialized = False
        self.running = False
        self.worker = None
        self.loop_wavelengths = []

        self.log_queue = queue.Queue()
        self.done_token = object()
        self.stop_event = threading.Event()
        self.return_event = threading.Event()

        self.inputs = {}
        self.build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.drain_log)

    def build_ui(self):
        form = ttk.LabelFrame(self, text="Experiment Parameters", padding=12)
        form.pack(fill="x", padx=12, pady=10)

        fields = [
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Total Distance mm", "TOTAL_DISTANCE_MM", config.TOTAL_DISTANCE_MM),
            ("Step Distance mm", "STEP_DISTANCE_MM", config.STEP_DISTANCE_MM),
            ("Step Speed mm/s", "STEP_SPEED_MM_S", config.STEP_SPEED_MM_S),
            ("Step Wait Seconds", "STEP_WAIT_SECONDS", config.STEP_WAIT_SECONDS),
            ("Trigger DO Index", "TRIGGER_DO_INDEX", config.TRIGGER_DO_INDEX),
            ("Trigger Pulse Seconds", "TRIGGER_PULSE_SECONDS", config.TRIGGER_PULSE_SECONDS),
            ("Wavelengths nm", "WAVELENGTHS", str(config.LASER_WAVELENGTH_NM)),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(form, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(form, width=34)
            entry.insert(0, str(value))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        ttk.Label(form, text="Direction").grid(row=len(fields), column=0, sticky="w", pady=4)
        self.direction_box = ttk.Combobox(form, values=DIRECTIONS, state="readonly", width=31)
        self.direction_box.set(self.default_direction())
        self.direction_box.grid(row=len(fields), column=1, sticky="ew", pady=4)
        form.columnconfigure(1, weight=1)

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
        self.log_text = tk.Text(log_frame, height=16, wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def default_direction(self):
        offset = config.STEP_OFFSET_MM
        if abs(offset[0]) > 0:
            return "x+" if offset[0] > 0 else "x-"
        if abs(offset[1]) > 0:
            return "y+" if offset[1] > 0 else "y-"
        if abs(offset[2]) > 0:
            return "z+" if offset[2] > 0 else "z-"
        return "x-"

    def read_parameters(self):
        speed_ratio = int(float(self.inputs["SPEED_RATIO"].get()))
        total_distance = float(self.inputs["TOTAL_DISTANCE_MM"].get())
        step_distance = float(self.inputs["STEP_DISTANCE_MM"].get())
        step_speed = float(self.inputs["STEP_SPEED_MM_S"].get())
        step_wait = float(self.inputs["STEP_WAIT_SECONDS"].get())
        do_index = int(float(self.inputs["TRIGGER_DO_INDEX"].get()))
        pulse_seconds = float(self.inputs["TRIGGER_PULSE_SECONDS"].get())
        wavelengths = [
            float(value.strip())
            for value in self.inputs["WAVELENGTHS"].get().split(",")
            if value.strip()
        ]
        direction = self.direction_box.get()

        if speed_ratio <= 0:
            raise ValueError("Speed Ratio must be greater than 0")
        if total_distance <= 0:
            raise ValueError("Total Distance mm must be greater than 0")
        if step_distance <= 0:
            raise ValueError("Step Distance mm must be greater than 0")
        if step_speed <= 0:
            raise ValueError("Step Speed mm/s must be greater than 0")
        if step_wait < 0:
            raise ValueError("Step Wait Seconds cannot be negative")
        if do_index <= 0:
            raise ValueError("Trigger DO Index must be greater than 0")
        if pulse_seconds <= 0:
            raise ValueError("Trigger Pulse Seconds must be greater than 0")
        if not wavelengths:
            raise ValueError("Enter at least one wavelength")
        if any(wavelength <= 0 for wavelength in wavelengths):
            raise ValueError("Each wavelength must be greater than 0")
        if direction not in DIRECTIONS:
            raise ValueError("Direction must be one of x+/x-/y+/y-/z+/z-")

        offsets = {
            "x+": [step_distance, 0, 0],
            "x-": [-step_distance, 0, 0],
            "y+": [0, step_distance, 0],
            "y-": [0, -step_distance, 0],
            "z+": [0, 0, step_distance],
            "z-": [0, 0, -step_distance],
        }

        config.SPEED_RATIO = speed_ratio
        config.TOTAL_DISTANCE_MM = total_distance
        config.STEP_DISTANCE_MM = step_distance
        config.STEP_OFFSET_MM = offsets[direction]
        config.STEP_SPEED_MM_S = step_speed
        config.STEP_WAIT_SECONDS = step_wait
        config.TRIGGER_DO_INDEX = do_index
        config.TRIGGER_PULSE_SECONDS = pulse_seconds
        config.LOOP_REPEAT_COUNT = len(wavelengths)
        config.LASER_WAVELENGTH_NM = wavelengths[0]
        self.loop_wavelengths = wavelengths

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
                    self.laser, self.dobot, self.feed_thread, self.saved_start_pose = experiment1.initialize()
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
        self.log("Starting experiment")

        self.worker = threading.Thread(target=self.run_experiment, daemon=True)
        self.worker.start()

    def run_experiment(self):
        try:
            with contextlib.redirect_stdout(self):
                step_count = calculate_step_count(config.TOTAL_DISTANCE_MM, config.STEP_DISTANCE_MM)
                print("Step count:", step_count)

                if get_robot_error(self.dobot):
                    self.return_to_saved_start()
                    return

                self.dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
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

                    for step_index in range(1, step_count + 1):
                        if self.stop_or_return_requested():
                            return
                        run_step(
                            self.dobot,
                            step_index,
                            config.STEP_OFFSET_MM,
                            config.SPEED_RATIO,
                            config.STEP_WAIT_SECONDS,
                            config.TRIGGER_DO_INDEX,
                            config.TRIGGER_PULSE_SECONDS,
                        )

                    turn_do_off(self.dobot, config.TRIGGER_DO_INDEX)
                    self.dobot.MoveLinearPoint(self.saved_start_pose, config.SPEED_RATIO)
                    if self.wait_seconds(2):
                        return
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            if self.laser is not None:
                self.laser.stop_safely()
            self.log("Experiment finished")
            self.log_queue.put(self.done_token)

    def wait_seconds(self, seconds):
        end_time = time.perf_counter() + seconds
        while time.perf_counter() < end_time:
            if self.stop_event.is_set() or self.return_event.is_set():
                self.return_to_saved_start()
                return True
            time.sleep(min(0.1, end_time - time.perf_counter()))
        return False

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


if __name__ == "__main__":
    ExperimentGui().mainloop()
