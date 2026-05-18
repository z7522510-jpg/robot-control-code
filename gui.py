import contextlib
import json
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import config


DIRECTIONS = ("x+", "x-", "y+", "y-", "z+", "z-")
SETTINGS_PATH = Path(__file__).with_name("ui_settings.json")


class QueueWriter:
    def __init__(self, log_queue):
        self.log_queue = log_queue

    def write(self, text):
        if text.strip():
            self.log_queue.put(text)

    def flush(self):
        pass


class ExperimentGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dobot Laser Experiment")
        self.geometry("780x680")

        self.laser = None
        self.dobot = None
        self.feed_thread = None
        self.saved_start_pose = None
        self.initialized = False
        self.worker = None
        self.loop_wavelengths = []

        self.done_token = object()
        self.log_queue = queue.Queue()
        self.stop_event = threading.Event()
        self.return_event = threading.Event()
        self.loop_count_var = tk.StringVar(value=str(config.LOOP_REPEAT_COUNT))

        self.inputs = {}
        self.wavelength_entries = []
        self.settings = self._load_settings()

        self._build_form()
        self._build_wavelengths()
        self._build_buttons()
        self._build_log()

        self.loop_count_var.trace_add("write", self._schedule_wavelength_update)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._drain_log)

    def _load_settings(self):
        if not SETTINGS_PATH.exists():
            return {}
        try:
            with SETTINGS_PATH.open("r", encoding="utf-8") as file:
                return json.load(file)
        except Exception:
            return {}

    def _save_settings(self, wavelengths):
        settings = {
            "SPEED_RATIO": self.inputs["SPEED_RATIO"].get(),
            "TOTAL_DISTANCE_MM": self.inputs["TOTAL_DISTANCE_MM"].get(),
            "STEP_DISTANCE_MM": self.inputs["STEP_DISTANCE_MM"].get(),
            "STEP_WAIT_SECONDS": self.inputs["STEP_WAIT_SECONDS"].get(),
            "TRIGGER_DO_INDEX": self.inputs["TRIGGER_DO_INDEX"].get(),
            "TRIGGER_PULSE_SECONDS": self.inputs["TRIGGER_PULSE_SECONDS"].get(),
            "LOOP_REPEAT_COUNT": self.inputs["LOOP_REPEAT_COUNT"].get(),
            "DIRECTION": self.direction_box.get(),
            "WAVELENGTHS": [str(wavelength) for wavelength in wavelengths],
        }
        with SETTINGS_PATH.open("w", encoding="utf-8") as file:
            json.dump(settings, file, indent=2)

    def _build_form(self):
        frame = ttk.LabelFrame(self, text="Experiment Parameters", padding=12)
        frame.pack(fill="x", padx=12, pady=10)

        fields = [
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Total Distance mm", "TOTAL_DISTANCE_MM", config.TOTAL_DISTANCE_MM),
            ("Step Distance mm", "STEP_DISTANCE_MM", config.STEP_DISTANCE_MM),
            ("Step Wait Seconds", "STEP_WAIT_SECONDS", config.STEP_WAIT_SECONDS),
            ("Trigger DO Index", "TRIGGER_DO_INDEX", config.TRIGGER_DO_INDEX),
            ("Trigger Pulse Seconds", "TRIGGER_PULSE_SECONDS", config.TRIGGER_PULSE_SECONDS),
            ("Loop Repeat Count", "LOOP_REPEAT_COUNT", config.LOOP_REPEAT_COUNT),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(frame, width=34)
            if key == "LOOP_REPEAT_COUNT":
                self.loop_count_var.set(str(self.settings.get(key, value)))
                entry.configure(textvariable=self.loop_count_var)
            else:
                entry.insert(0, str(self.settings.get(key, value)))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        ttk.Label(frame, text="Direction").grid(row=len(fields), column=0, sticky="w", pady=4)
        self.direction_box = ttk.Combobox(frame, values=DIRECTIONS, state="readonly", width=31)
        self.direction_box.set(self.settings.get("DIRECTION", self._default_direction()))
        self.direction_box.grid(row=len(fields), column=1, sticky="ew", pady=4)

        frame.columnconfigure(1, weight=1)

    def _build_wavelengths(self):
        self.wavelength_frame = ttk.LabelFrame(self, text="Loop Wavelengths", padding=12)
        self.wavelength_frame.pack(fill="x", padx=12, pady=(0, 10))
        self._update_wavelength_entries()

    def _build_buttons(self):
        frame = ttk.Frame(self, padding=(12, 0))
        frame.pack(fill="x")

        self.init_button = ttk.Button(frame, text="Initialize Robot and Laser", command=self.initialize_devices)
        self.start_button = ttk.Button(frame, text="Start Experiment", command=self.start_experiment, state="disabled")
        self.stop_button = ttk.Button(frame, text="Stop", command=self.stop_experiment, state="disabled")
        self.return_button = ttk.Button(frame, text="Return To Start", command=self.return_to_start, state="disabled")

        self.init_button.pack(side="left", padx=(0, 8))
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button.pack(side="left", padx=(0, 8))
        self.return_button.pack(side="left")

    def _build_log(self):
        frame = ttk.LabelFrame(self, text="Log", padding=8)
        frame.pack(fill="both", expand=True, padx=12, pady=10)

        self.log_text = tk.Text(frame, height=16, wrap="word")
        scrollbar = ttk.Scrollbar(frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

    def _default_direction(self):
        offset = config.STEP_OFFSET_MM
        if abs(offset[0]) > 0:
            return "x+" if offset[0] > 0 else "x-"
        if abs(offset[1]) > 0:
            return "y+" if offset[1] > 0 else "y-"
        if abs(offset[2]) > 0:
            return "z+" if offset[2] > 0 else "z-"
        return "x-"

    def _schedule_wavelength_update(self, *_args):
        self.after_idle(self._update_wavelength_entries)

    def _update_wavelength_entries(self):
        old_values = [entry.get() for entry in self.wavelength_entries]
        saved_values = self.settings.get("WAVELENGTHS", [])
        try:
            loop_count = int(float(self.loop_count_var.get()))
        except ValueError:
            return
        if loop_count <= 0 or loop_count > 100:
            return

        for child in self.wavelength_frame.winfo_children():
            child.destroy()
        self.wavelength_entries = []

        for index in range(loop_count):
            if index < len(old_values):
                value = old_values[index]
            elif index < len(saved_values):
                value = saved_values[index]
            else:
                value = str(config.LASER_WAVELENGTH_NM)
            ttk.Label(self.wavelength_frame, text=f"Loop {index + 1} Wavelength nm").grid(
                row=index,
                column=0,
                sticky="w",
                pady=3,
            )
            entry = ttk.Entry(self.wavelength_frame, width=34)
            entry.insert(0, value)
            entry.grid(row=index, column=1, sticky="ew", pady=3)
            self.wavelength_entries.append(entry)

        self.wavelength_frame.columnconfigure(1, weight=1)

    def read_parameters(self):
        speed_ratio = int(float(self.inputs["SPEED_RATIO"].get()))
        total_distance = float(self.inputs["TOTAL_DISTANCE_MM"].get())
        step_distance = float(self.inputs["STEP_DISTANCE_MM"].get())
        step_wait = float(self.inputs["STEP_WAIT_SECONDS"].get())
        do_index = int(float(self.inputs["TRIGGER_DO_INDEX"].get()))
        pulse_seconds = float(self.inputs["TRIGGER_PULSE_SECONDS"].get())
        loop_count = int(float(self.inputs["LOOP_REPEAT_COUNT"].get()))
        direction = self.direction_box.get()
        wavelengths = [float(entry.get()) for entry in self.wavelength_entries]

        if speed_ratio <= 0:
            raise ValueError("Speed Ratio must be greater than 0")
        if total_distance <= 0:
            raise ValueError("Total Distance mm must be greater than 0")
        if step_distance <= 0:
            raise ValueError("Step Distance mm must be greater than 0")
        if step_wait < 0:
            raise ValueError("Step Wait Seconds cannot be negative")
        if do_index <= 0:
            raise ValueError("Trigger DO Index must be greater than 0")
        if pulse_seconds <= 0:
            raise ValueError("Trigger Pulse Seconds must be greater than 0")
        if loop_count <= 0:
            raise ValueError("Loop Repeat Count must be greater than 0")
        if len(wavelengths) != loop_count:
            raise ValueError("Loop wavelength count must match Loop Repeat Count")
        if any(wavelength <= 0 for wavelength in wavelengths):
            raise ValueError("Each loop wavelength must be greater than 0")
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
        config.STEP_WAIT_SECONDS = step_wait
        config.TRIGGER_DO_INDEX = do_index
        config.TRIGGER_PULSE_SECONDS = pulse_seconds
        config.LOOP_REPEAT_COUNT = loop_count
        config.LASER_WAVELENGTH_NM = wavelengths[0]
        self._save_settings(wavelengths)
        return wavelengths

    def initialize_devices(self):
        if self.worker and self.worker.is_alive():
            return

        try:
            self.loop_wavelengths = self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.log("Initializing robot and laser")
        self._set_initializing(True)
        self.worker = threading.Thread(target=self._initialize_devices, daemon=True)
        self.worker.start()

    def _initialize_devices(self):
        try:
            with contextlib.redirect_stdout(QueueWriter(self.log_queue)):
                import experiment1

                self.laser, self.dobot, self.feed_thread, self.saved_start_pose = experiment1.initialize()

            self.initialized = True
            self.log_queue.put("Robot and laser initialization complete")
        except ModuleNotFoundError as error:
            self._log_missing_module(error)
        except Exception as error:
            self.log_queue.put(f"ERROR: {error}")
        finally:
            self.log_queue.put(self.done_token)

    def start_experiment(self):
        if self.worker and self.worker.is_alive():
            return

        try:
            self.loop_wavelengths = self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return
        if not self.initialized:
            messagebox.showerror("Not Initialized", "Initialize robot and laser before starting.")
            return

        self.stop_event.clear()
        self.return_event.clear()
        self.log("Starting experiment")
        self._set_running(True)
        self.worker = threading.Thread(target=self._run_experiment, daemon=True)
        self.worker.start()

    def _run_experiment(self):
        try:
            with contextlib.redirect_stdout(QueueWriter(self.log_queue)):
                self._run_experiment1_flow()
        except ModuleNotFoundError as error:
            self._log_missing_module(error)
        except Exception as error:
            self.log_queue.put(f"ERROR: {error}")
        finally:
            self.log_queue.put("Experiment finished")
            self.log_queue.put(self.done_token)

    def _run_experiment1_flow(self):
        import experiment1
        from Dobot import calculate_step_count, get_robot_error, run_step, turn_do_off

        step_count = calculate_step_count(config.TOTAL_DISTANCE_MM, config.STEP_DISTANCE_MM)
        print("Step count:", step_count)

        try:
            if get_robot_error(self.dobot):
                experiment1.stop_laser_and_return(self.laser, self.dobot, self.saved_start_pose)
                return

            self.laser.run()
            print("Laser RUN")
            if self._wait_or_stop(5):
                return

            for loop_index, wavelength in enumerate(self.loop_wavelengths, start=1):
                if self._should_stop_or_return():
                    return
                if get_robot_error(self.dobot):
                    experiment1.stop_laser_and_return(self.laser, self.dobot, self.saved_start_pose)
                    return

                print(f"Loop {loop_index}/{len(self.loop_wavelengths)}")
                self.laser.set_wavelength(wavelength)
                if self._wait_or_stop(2):
                    return

                for step_index in range(1, step_count + 1):
                    if self._should_stop_or_return():
                        return
                    if get_robot_error(self.dobot):
                        experiment1.stop_laser_and_return(self.laser, self.dobot, self.saved_start_pose)
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

                # One error check after the whole scan loop (covers the last
                # step too); the per-step check is at the top of the next loop.
                if get_robot_error(self.dobot):
                    experiment1.stop_laser_and_return(self.laser, self.dobot, self.saved_start_pose)
                    return

                turn_do_off(self.dobot, config.TRIGGER_DO_INDEX)
                self.dobot.MoveLinearPoint(self.saved_start_pose, config.SPEED_RATIO)
                if self._wait_or_stop(2):
                    return
        finally:
            self.laser.stop_safely()

    def _abort_and_return(self):
        # Stop the laser and move the robot back to the saved start pose.
        # Used by Stop, Return To Start, and window close.
        #
        # Abort is only processed between steps / during waits, so the robot is
        # idle here. Use a direct MovL (same as the end-of-loop return) instead
        # of stop_and_return: this firmware rejects MovL right after Stop().
        if self.laser is not None:
            try:
                self.laser.stop_safely()
            except Exception:
                pass
        if self.dobot is not None and self.saved_start_pose is not None:
            from Dobot import turn_do_off

            try:
                turn_do_off(self.dobot, config.TRIGGER_DO_INDEX)
            except Exception:
                pass
            self.dobot.MoveLinearPoint(self.saved_start_pose, config.SPEED_RATIO)

    def _wait_or_stop(self, seconds):
        if self.stop_event.wait(seconds):
            self._abort_and_return()
            return True
        return False

    def _should_stop_or_return(self):
        # Stop and Return To Start both return the robot to the start pose.
        if self.return_event.is_set() or self.stop_event.is_set():
            self._abort_and_return()
            return True
        return False

    def stop_experiment(self):
        self.log("Stop requested - laser off and returning to start pose")
        self.stop_event.set()

    def return_to_start(self):
        self.log("Return to start requested")
        self.return_event.set()

    def _set_initializing(self, initializing):
        self.init_button.configure(state="disabled" if initializing else "normal")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="disabled")
        self.return_button.configure(state="disabled")

    def _set_running(self, running):
        self.init_button.configure(state="disabled" if running else "normal")
        self.start_button.configure(state="disabled" if running else "normal")
        self.stop_button.configure(state="normal" if running else "disabled")
        self.return_button.configure(state="normal" if running else "disabled")

    def _finish_worker(self):
        self.init_button.configure(state="normal")
        self.start_button.configure(state="normal" if self.initialized else "disabled")
        self.stop_button.configure(state="disabled")
        self.return_button.configure(state="disabled")

    def _log_missing_module(self, error):
        if error.name == "numpy":
            self.log_queue.put("ERROR: Missing dependency 'numpy'. Install it with: pip install numpy")
        else:
            self.log_queue.put(f"ERROR: Missing module '{error.name}'")

    def log(self, text):
        self.log_queue.put(text)

    def _drain_log(self):
        while True:
            try:
                text = self.log_queue.get_nowait()
            except queue.Empty:
                break
            if text is self.done_token:
                self._finish_worker()
                continue
            self.log_text.insert("end", text.rstrip() + "\n")
            self.log_text.see("end")
        self.after(100, self._drain_log)

    def on_close(self):
        # Ask any running experiment to stop and return to the start pose,
        # then wait for it to finish before disconnecting.
        self.stop_event.set()
        self.return_event.set()
        if self.worker is not None and self.worker.is_alive():
            self.worker.join(timeout=120)
        elif self.dobot is not None and self.saved_start_pose is not None:
            # No experiment running: return to start directly before cleanup.
            try:
                self._abort_and_return()
            except Exception:
                pass
        if self.laser is not None:
            try:
                self.laser.close()
            except Exception:
                pass
        if self.dobot is not None:
            try:
                from Dobot import disconnect_robot

                disconnect_robot(self.dobot)
            except Exception:
                pass
        self.destroy()


if __name__ == "__main__":
    ExperimentGui().mainloop()
