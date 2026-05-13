import ast
import contextlib
import queue
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import config
import experiment


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
        self.geometry("760x560")

        self.log_queue = queue.Queue()
        self.worker = None
        self.stop_event = threading.Event()
        self.return_event = threading.Event()

        self.inputs = {}
        self._build_form()
        self._build_buttons()
        self._build_log()
        self.after(100, self._drain_log)

    def _build_form(self):
        frame = ttk.LabelFrame(self, text="Experiment Parameters", padding=12)
        frame.pack(fill="x", padx=12, pady=10)

        fields = [
            ("Laser Wavelength nm", "LASER_WAVELENGTH_NM", config.LASER_WAVELENGTH_NM),
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Total Distance mm", "TOTAL_DISTANCE_MM", config.TOTAL_DISTANCE_MM),
            ("Step Offset XYZ mm", "STEP_OFFSET_MM", config.STEP_OFFSET_MM),
            ("Step Wait Seconds", "STEP_WAIT_SECONDS", config.STEP_WAIT_SECONDS),
            ("Loop Repeat Count", "LOOP_REPEAT_COUNT", config.LOOP_REPEAT_COUNT),
        ]

        for row, (label, key, value) in enumerate(fields):
            ttk.Label(frame, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(frame, width=32)
            entry.insert(0, str(value))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        frame.columnconfigure(1, weight=1)

    def _build_buttons(self):
        frame = ttk.Frame(self, padding=(12, 0))
        frame.pack(fill="x")

        self.start_button = ttk.Button(frame, text="Start Experiment", command=self.start_experiment)
        self.stop_button = ttk.Button(frame, text="Stop", command=self.stop_experiment, state="disabled")
        self.return_button = ttk.Button(frame, text="Return To Start", command=self.return_to_start, state="disabled")

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

    def read_parameters(self):
        offset = ast.literal_eval(self.inputs["STEP_OFFSET_MM"].get())
        if not isinstance(offset, (list, tuple)) or len(offset) != 3:
            raise ValueError("Step Offset XYZ mm must be a list like [0, -2, 0]")

        config.LASER_WAVELENGTH_NM = float(self.inputs["LASER_WAVELENGTH_NM"].get())
        config.SPEED_RATIO = int(float(self.inputs["SPEED_RATIO"].get()))
        config.TOTAL_DISTANCE_MM = float(self.inputs["TOTAL_DISTANCE_MM"].get())
        config.STEP_OFFSET_MM = [float(value) for value in offset]
        config.STEP_WAIT_SECONDS = float(self.inputs["STEP_WAIT_SECONDS"].get())
        config.LOOP_REPEAT_COUNT = int(float(self.inputs["LOOP_REPEAT_COUNT"].get()))

    def start_experiment(self):
        if self.worker is not None and self.worker.is_alive():
            return

        try:
            self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.stop_event.clear()
        self.return_event.clear()
        self._set_running(True)
        self.log("Starting experiment")

        self.worker = threading.Thread(target=self._run_experiment, daemon=True)
        self.worker.start()

    def _run_experiment(self):
        try:
            with contextlib.redirect_stdout(QueueWriter(self.log_queue)):
                experiment.run_experiment(
                    require_confirm=False,
                    stop_event=self.stop_event,
                    return_event=self.return_event,
                )
        except Exception as error:
            self.log_queue.put(f"ERROR: {error}")
        finally:
            self.log_queue.put("Experiment finished")
            self.after(0, lambda: self._set_running(False))

    def stop_experiment(self):
        self.log("Stop requested")
        self.stop_event.set()

    def return_to_start(self):
        self.log("Return to start requested")
        self.return_event.set()

    def _set_running(self, running):
        self.start_button.configure(state="disabled" if running else "normal")
        self.stop_button.configure(state="normal" if running else "disabled")
        self.return_button.configure(state="normal" if running else "disabled")

    def log(self, text):
        self.log_queue.put(text)

    def _drain_log(self):
        while True:
            try:
                text = self.log_queue.get_nowait()
            except queue.Empty:
                break
            self.log_text.insert("end", text.rstrip() + "\n")
            self.log_text.see("end")
        self.after(100, self._drain_log)


if __name__ == "__main__":
    ExperimentGui().mainloop()
