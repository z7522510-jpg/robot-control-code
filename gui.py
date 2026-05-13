import ast
import contextlib
import math
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import config


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
        self.geometry("760x620")

        self.dobot = None
        self.laser = None
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

        self._build_form()
        self._build_wavelengths()
        self._build_buttons()
        self._build_log()

        self.loop_count_var.trace_add("write", self._schedule_wavelength_update)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self._drain_log)

    def _build_form(self):
        frame = ttk.LabelFrame(self, text="Experiment Parameters", padding=12)
        frame.pack(fill="x", padx=12, pady=10)

        fields = [
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Total Distance mm", "TOTAL_DISTANCE_MM", config.TOTAL_DISTANCE_MM),
            ("Step Offset XYZ mm", "STEP_OFFSET_MM", config.STEP_OFFSET_MM),
            ("Step Wait Seconds", "STEP_WAIT_SECONDS", config.STEP_WAIT_SECONDS),
            ("Loop Repeat Count", "LOOP_REPEAT_COUNT", config.LOOP_REPEAT_COUNT),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(frame, width=34)
            if key == "LOOP_REPEAT_COUNT":
                entry.configure(textvariable=self.loop_count_var)
            else:
                entry.insert(0, str(value))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        frame.columnconfigure(1, weight=1)

    def _build_wavelengths(self):
        self.wavelength_frame = ttk.LabelFrame(self, text="Loop Wavelengths", padding=12)
        self.wavelength_frame.pack(fill="x", padx=12, pady=(0, 10))
        self._update_wavelength_entries()

    def _build_buttons(self):
        frame = ttk.Frame(self, padding=(12, 0))
        frame.pack(fill="x")

        self.init_button = ttk.Button(
            frame,
            text="Initialize Robot and Laser",
            command=self.initialize_devices,
        )
        self.start_button = ttk.Button(
            frame,
            text="Start Experiment",
            command=self.start_experiment,
            state="disabled",
        )
        self.stop_button = ttk.Button(
            frame,
            text="Stop",
            command=self.stop_experiment,
            state="disabled",
        )
        self.return_button = ttk.Button(
            frame,
            text="Return To Start",
            command=self.return_to_start,
            state="disabled",
        )

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

    def _schedule_wavelength_update(self, *_args):
        self.after_idle(self._update_wavelength_entries)

    def _update_wavelength_entries(self):
        old_values = [entry.get() for entry in self.wavelength_entries]
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
            value = old_values[index] if index < len(old_values) else str(config.LASER_WAVELENGTH_NM)
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
        offset = ast.literal_eval(self.inputs["STEP_OFFSET_MM"].get())
        if not isinstance(offset, (list, tuple)) or len(offset) != 3:
            raise ValueError("Step Offset XYZ mm must look like [0, -2, 0]")

        speed_ratio = int(float(self.inputs["SPEED_RATIO"].get()))
        total_distance = float(self.inputs["TOTAL_DISTANCE_MM"].get())
        step_offset = [float(value) for value in offset]
        step_wait = float(self.inputs["STEP_WAIT_SECONDS"].get())
        loop_count = int(float(self.inputs["LOOP_REPEAT_COUNT"].get()))
        wavelengths = [float(entry.get()) for entry in self.wavelength_entries]

        if speed_ratio <= 0:
            raise ValueError("Speed Ratio must be greater than 0")
        if total_distance <= 0:
            raise ValueError("Total Distance mm must be greater than 0")
        if math.sqrt(sum(value * value for value in step_offset)) <= 1e-12:
            raise ValueError("Step Offset XYZ mm must include movement")
        if step_wait < 0:
            raise ValueError("Step Wait Seconds cannot be negative")
        if loop_count <= 0:
            raise ValueError("Loop Repeat Count must be greater than 0")
        if len(wavelengths) != loop_count:
            raise ValueError("Loop wavelength count must match Loop Repeat Count")
        if any(wavelength <= 0 for wavelength in wavelengths):
            raise ValueError("Each loop wavelength must be greater than 0")

        config.SPEED_RATIO = speed_ratio
        config.TOTAL_DISTANCE_MM = total_distance
        config.STEP_OFFSET_MM = step_offset
        config.STEP_WAIT_SECONDS = step_wait
        config.LOOP_REPEAT_COUNT = loop_count
        config.LASER_WAVELENGTH_NM = wavelengths[0]
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
                import experiment
                from Dobot import initialize_robot
                from Laser import connect_laser

                self.dobot, _feed_thread, _original_pose = initialize_robot(
                    config.DOBOT_IP,
                    config.SPEED_RATIO,
                    dobot=self.dobot,
                )

                if self.laser is None:
                    self.laser = connect_laser(config.LASER_DLL_PATH)
                self.laser.initialize_laser(config.LASER_WAVELENGTH_NM)

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
                import experiment

                experiment.run_initialized_experiment(
                    dobot=self.dobot,
                    laser=self.laser,
                    loop_wavelengths=self.loop_wavelengths,
                    stop_event=self.stop_event,
                    return_event=self.return_event,
                )
        except ModuleNotFoundError as error:
            self._log_missing_module(error)
        except Exception as error:
            self.log_queue.put(f"ERROR: {error}")
        finally:
            self.log_queue.put("Experiment finished")
            self.log_queue.put(self.done_token)

    def stop_experiment(self):
        self.log("Stop requested")
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
        self.stop_event.set()
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
