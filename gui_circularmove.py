import contextlib
import json
import queue
import threading
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox, ttk

import circularmove
import config
from Dobot import disconnect_robot, get_robot_error


SETTINGS_PATH = Path(__file__).with_name("circularmove_ui_settings.json")


class CircularMoveGui(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Circular Scanning")
        self.geometry("980x720")

        self.laser = None
        self.dobot = None
        self.feed_thread = None
        self.worker = None
        self.devices_ready = False
        self.prepared = False
        self.running = False
        self.radius_returned = True
        self.stop_event = threading.Event()
        self.log_queue = queue.Queue()
        self.done_token = object()
        self.report_path = None
        self.real_start_pose = None
        self.radius_pose = None
        self.center_pose = None
        self.circle_tool_frame = None
        self.poses = []

        self._busy = False
        self.settings = self.load_settings()
        self.inputs = {}
        self.rz_var = tk.DoubleVar(value=float(self.settings.get("CIRCLE_RZ_DEG", config.CIRCLE_RZ_DEG)))
        self.tool_frame_var = tk.StringVar(value=config.TOOL_FRAME)
        self.circle_frame_var = tk.StringVar(value="Not prepared")
        self.center_pose_var = tk.StringVar(value="Not prepared")
        self.progress_var = tk.StringVar(value="0/0")

        self.build_ui()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(100, self.drain_log)

    def build_ui(self):
        main = ttk.Frame(self, padding=12)
        main.pack(fill="both", expand=True)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main)
        left.grid(row=0, column=0, sticky="nsw", padx=(0, 12))

        right = ttk.Frame(main)
        right.grid(row=0, column=1, sticky="nsew")
        right.rowconfigure(2, weight=1)
        right.columnconfigure(0, weight=1)

        params = ttk.LabelFrame(left, text="Scan Inputs", padding=12)
        params.pack(fill="x")

        fields = [
            ("Radius mm", "CIRCLE_RADIUS_MM", config.CIRCLE_RADIUS_MM),
            (
                "Subcutaneous Distance mm",
                "SUBCUTANEOUS_SCAN_DISTANCE_MM",
                config.SUBCUTANEOUS_SCAN_DISTANCE_MM,
            ),
            ("Circle Arc deg", "CIRCLE_ARC_DEG", config.CIRCLE_ARC_DEG),
            ("Total Steps", "CIRCLE_TOTAL_STEPS", config.CIRCLE_TOTAL_STEPS),
            ("Speed Ratio", "SPEED_RATIO", config.SPEED_RATIO),
            ("Acceleration Ratio", "CIRCLE_ACCELERATION_RATIO", config.CIRCLE_ACCELERATION_RATIO),
            ("Velocity Ratio", "CIRCLE_VELOCITY_RATIO", config.CIRCLE_VELOCITY_RATIO),
            ("CP", "CIRCLE_CP", config.CIRCLE_CP),
            ("Trigger DO Index", "TRIGGER_DO_INDEX", config.TRIGGER_DO_INDEX),
            ("Trigger Pulse Seconds", "TRIGGER_PULSE_SECONDS", config.TRIGGER_PULSE_SECONDS),
            ("Tool Frame", "TOOL_FRAME", config.TOOL_FRAME),
        ]

        for row, (label, key, default) in enumerate(fields):
            ttk.Label(params, text=label).grid(row=row, column=0, sticky="w", pady=4)
            entry = ttk.Entry(params, width=34)
            entry.insert(0, str(self.settings.get(key, default)))
            entry.grid(row=row, column=1, sticky="ew", pady=4)
            self.inputs[key] = entry

        params.columnconfigure(1, weight=1)

        rz_frame = ttk.LabelFrame(left, text="Start Rz", padding=12)
        rz_frame.pack(fill="x", pady=(10, 0))

        rz_move_frame = ttk.Frame(rz_frame)
        rz_move_frame.pack(fill="x")
        ttk.Label(rz_move_frame, text="Rz deg:").pack(side="left", padx=(0, 8))
        rz_entry = ttk.Entry(rz_move_frame, width=10, textvariable=self.rz_var)
        rz_entry.pack(side="left", padx=(0, 8))
        self.move_rz_button = ttk.Button(
            rz_move_frame, text="Move to Rz", command=self.move_to_rz, state="disabled"
        )
        self.move_rz_button.pack(side="left")

        jog_frame = ttk.Frame(rz_frame)
        jog_frame.pack(fill="x", pady=(8, 0))
        ttk.Label(jog_frame, text="Jog (hold):").pack(side="left", padx=(0, 8))
        self.rz_minus_button = ttk.Button(jog_frame, text="Rz −", width=6, state="disabled")
        self.rz_plus_button = ttk.Button(jog_frame, text="Rz +", width=6, state="disabled")
        self.rz_minus_button.pack(side="left", padx=(0, 8))
        self.rz_plus_button.pack(side="left")
        self.rz_minus_button.bind("<ButtonPress-1>", lambda _event: self.jog_rz_start("-"))
        self.rz_minus_button.bind("<ButtonRelease-1>", lambda _event: self.jog_rz_stop())
        self.rz_plus_button.bind("<ButtonPress-1>", lambda _event: self.jog_rz_start("+"))
        self.rz_plus_button.bind("<ButtonRelease-1>", lambda _event: self.jog_rz_stop())

        buttons = ttk.Frame(left)
        buttons.pack(fill="x", pady=(12, 0))
        self.init_button = ttk.Button(buttons, text="Initialize", command=self.initialize_devices)
        self.level_xz_button = ttk.Button(buttons, text="Level XZ", command=self.level_xz, state="disabled")
        self.set_radius_button = ttk.Button(buttons, text="Set Radius", command=self.set_radius, state="disabled")
        self.start_button = ttk.Button(buttons, text="Start Scan", command=self.start_scan, state="disabled")
        self.stop_button = ttk.Button(buttons, text="Stop", command=self.stop_scan, state="disabled")
        self.exit_button = ttk.Button(buttons, text="Exit", command=self.on_close)
        self.init_button.pack(side="left", padx=(0, 8))
        self.level_xz_button.pack(side="left", padx=(0, 8))
        self.set_radius_button.pack(side="left", padx=(0, 8))
        self.start_button.pack(side="left", padx=(0, 8))
        self.stop_button.pack(side="left", padx=(0, 8))
        self.exit_button.pack(side="left")

        status = ttk.LabelFrame(right, text="Status", padding=12)
        status.grid(row=0, column=0, sticky="ew")
        status.columnconfigure(1, weight=1)

        status_rows = [
            ("Tool Frame", self.tool_frame_var),
            ("Circle Center Tool Frame", self.circle_frame_var),
            ("Central Circle Point", self.center_pose_var),
            ("Progress", self.progress_var),
        ]
        for row, (label, var) in enumerate(status_rows):
            ttk.Label(status, text=label).grid(row=row, column=0, sticky="nw", pady=4)
            ttk.Label(status, textvariable=var, wraplength=520).grid(
                row=row, column=1, sticky="ew", pady=4
            )

        log_frame = ttk.LabelFrame(right, text="Log", padding=8)
        log_frame.grid(row=2, column=0, sticky="nsew", pady=(10, 0))
        log_frame.rowconfigure(0, weight=1)
        log_frame.columnconfigure(0, weight=1)
        self.log_text = tk.Text(log_frame, height=18, wrap="word")
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self.log_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        self.update_static_status()

    def load_settings(self):
        try:
            with SETTINGS_PATH.open("r", encoding="utf-8") as file:
                return json.load(file)
        except Exception:
            return {}

    def save_settings(self):
        settings = {key: entry.get() for key, entry in self.inputs.items()}
        settings["CIRCLE_RZ_DEG"] = str(self.rz_var.get())
        with SETTINGS_PATH.open("w", encoding="utf-8") as file:
            json.dump(settings, file, indent=2)

    def read_parameters(self):
        values = {key: entry.get().strip() for key, entry in self.inputs.items()}

        config.CIRCLE_RADIUS_MM = float(values["CIRCLE_RADIUS_MM"])
        config.SUBCUTANEOUS_SCAN_DISTANCE_MM = float(values["SUBCUTANEOUS_SCAN_DISTANCE_MM"])
        config.CIRCLE_ARC_DEG = float(values["CIRCLE_ARC_DEG"])
        config.CIRCLE_TOTAL_STEPS = int(float(values["CIRCLE_TOTAL_STEPS"]))
        config.SPEED_RATIO = int(float(values["SPEED_RATIO"]))
        config.CIRCLE_ACCELERATION_RATIO = int(float(values["CIRCLE_ACCELERATION_RATIO"]))
        config.CIRCLE_VELOCITY_RATIO = int(float(values["CIRCLE_VELOCITY_RATIO"]))
        config.CIRCLE_CP = int(float(values["CIRCLE_CP"]))
        config.TRIGGER_DO_INDEX = int(float(values["TRIGGER_DO_INDEX"]))
        config.TRIGGER_PULSE_SECONDS = float(values["TRIGGER_PULSE_SECONDS"])
        config.TOOL_FRAME = values["TOOL_FRAME"]
        config.CIRCLE_RZ_DEG = float(self.rz_var.get())

        if config.CIRCLE_RADIUS_MM <= 0:
            raise ValueError("Radius mm must be greater than 0")
        if config.SUBCUTANEOUS_SCAN_DISTANCE_MM <= 0:
            raise ValueError("Subcutaneous Distance mm must be greater than 0")
        if config.CIRCLE_ARC_DEG <= 0:
            raise ValueError("Circle Arc deg must be greater than 0")
        if config.CIRCLE_TOTAL_STEPS <= 0:
            raise ValueError("Total Steps must be greater than 0")
        if config.SPEED_RATIO <= 0:
            raise ValueError("Speed Ratio must be greater than 0")
        if config.CIRCLE_ACCELERATION_RATIO <= 0 or config.CIRCLE_VELOCITY_RATIO <= 0:
            raise ValueError("Acceleration and Velocity Ratio must be greater than 0")
        if config.CIRCLE_CP < 0:
            raise ValueError("CP cannot be negative")
        if config.TRIGGER_DO_INDEX <= 0:
            raise ValueError("Trigger DO Index must be greater than 0")
        if config.TRIGGER_PULSE_SECONDS <= 0:
            raise ValueError("Trigger Pulse Seconds must be greater than 0")
        if len(circularmove._tool_frame_values(config.TOOL_FRAME)) != 6:
            raise ValueError("Tool Frame must have 6 values")

        self.save_settings()
        self.update_static_status()

    def update_static_status(self):
        self.tool_frame_var.set(config.TOOL_FRAME)
        try:
            self.circle_frame_var.set(circularmove.get_circle_center_tool_frame())
        except Exception:
            self.circle_frame_var.set("Invalid tool frame")

    def initialize_devices(self):
        if self.worker and self.worker.is_alive():
            return
        try:
            self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.stop_event.clear()
        self.set_busy(True)
        self.progress_var.set("0/0")
        self.log("Initializing devices")
        self.worker = threading.Thread(target=self.run_initialize_devices, daemon=True)
        self.worker.start()

    def run_initialize_devices(self):
        try:
            with contextlib.redirect_stdout(self):
                start_time = datetime.now()
                self.report_path = circularmove.create_current_pose_report(start_time)
                (
                    self.laser,
                    self.dobot,
                    self.feed_thread,
                    self.real_start_pose,
                ) = circularmove.initialize_devices(self.report_path)
                self.devices_ready = True
                print("Devices initialized. Press Set Radius to move the arm.")
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            self.log_queue.put(self.done_token)

    def set_radius(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.devices_ready or self.dobot is None:
            messagebox.showerror("Not Initialized", "Initialize devices before setting radius.")
            return
        try:
            self.read_parameters()
        except Exception as error:
            messagebox.showerror("Invalid Parameters", str(error))
            return

        self.set_busy(True)
        self.log("Setting radius (moving arm)")
        self.worker = threading.Thread(target=self.run_set_radius, daemon=True)
        self.worker.start()

    def run_set_radius(self):
        try:
            with contextlib.redirect_stdout(self):
                (
                    self.radius_pose,
                    self.center_pose,
                    self.circle_tool_frame,
                ) = circularmove.set_radius(self.dobot, self.report_path)

                self.center_pose = list(self.center_pose)
                self.center_pose[5] = config.CIRCLE_RZ_DEG
                self.queue_status("center", self.center_pose)

                self.poses = circularmove.generate_tool_center_circle_poses(
                    self.center_pose,
                    total_steps=config.CIRCLE_TOTAL_STEPS,
                    arc_deg=config.CIRCLE_ARC_DEG,
                )
                self.queue_status("progress", f"0/{len(self.poses)}")
                self.prepared = True
                print("Radius set. Ready to scan.")
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            self.log_queue.put(self.done_token)

    def level_xz(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.devices_ready or self.dobot is None:
            messagebox.showerror("Not Initialized", "Initialize devices before leveling.")
            return

        self.set_busy(True)
        self.log("Leveling XZ plane (rx=180, ry=0)")
        self.worker = threading.Thread(target=self.run_level_xz, daemon=True)
        self.worker.start()

    def run_level_xz(self):
        try:
            with contextlib.redirect_stdout(self):
                circularmove.level_xz_plane(self.dobot)
                print("XZ plane leveled (rx=180, ry=0)")
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            self.log_queue.put(self.done_token)

    def start_scan(self):
        if self.worker and self.worker.is_alive():
            return
        if not self.prepared or self.dobot is None or not self.poses:
            messagebox.showerror("Not Initialized", "Initialize scanning before starting.")
            return

        self.stop_event.clear()
        self.running = True
        self.radius_returned = False
        self.set_busy(True)
        self.log("Starting circular scan")
        self.worker = threading.Thread(target=self.run_scan, daemon=True)
        self.worker.start()

    def run_scan(self):
        try:
            with contextlib.redirect_stdout(self):
                total = len(self.poses)
                self.queue_status("progress", f"0/{total}")

                if get_robot_error(self.dobot):
                    self.return_to_radius_pose()
                    return

                start_pose = self.poses[0]
                print("Move to start pose:", start_pose)
                circularmove.run_step(
                    self.dobot,
                    start_pose,
                    user=config.CIRCLE_USER_INDEX,
                    tool=config.CIRCLE_TOOL_INDEX,
                    acceleration=config.CIRCLE_ACCELERATION_RATIO,
                    velocity=config.CIRCLE_VELOCITY_RATIO,
                    cp=config.CIRCLE_CP,
                    circle_tool_frame=self.circle_tool_frame,
                    stop_event=self.stop_event,
                    trigger_do_index=config.TRIGGER_DO_INDEX,
                    trigger_pulse_seconds=config.TRIGGER_PULSE_SECONDS,
                )
                circularmove.report_current_pose(self.dobot, self.report_path, "start pose", start_pose)
                self.queue_status("progress", f"1/{total}")

                for index, pose in enumerate(self.poses[1:], start=2):
                    if self.stop_event.is_set():
                        break
                    if get_robot_error(self.dobot):
                        self.return_to_radius_pose()
                        return

                    print(f"Circle point {index}/{total}:", pose)
                    circularmove.run_step(
                        self.dobot,
                        pose,
                        user=config.CIRCLE_USER_INDEX,
                        tool=config.CIRCLE_TOOL_INDEX,
                        acceleration=config.CIRCLE_ACCELERATION_RATIO,
                        velocity=config.CIRCLE_VELOCITY_RATIO,
                        cp=config.CIRCLE_CP,
                        circle_tool_frame=self.circle_tool_frame,
                        stop_event=self.stop_event,
                        trigger_do_index=config.TRIGGER_DO_INDEX,
                        trigger_pulse_seconds=config.TRIGGER_PULSE_SECONDS,
                    )
                    circularmove.report_current_pose(
                        self.dobot,
                        self.report_path,
                        f"circle point {index}/{total}",
                        pose,
                    )
                    self.queue_status("progress", f"{index}/{total}")

                self.return_to_radius_pose()
                circularmove.report_current_pose(
                    self.dobot,
                    self.report_path,
                    "return radius_adjusted_pose",
                    self.radius_pose,
                )
        except Exception as error:
            if self.stop_event.is_set():
                self.log(f"Scan stopped: {error}")
                try:
                    self.return_to_radius_pose()
                except Exception as return_error:
                    self.log(f"ERROR: {return_error}")
            else:
                self.log(f"ERROR: {error}")
        finally:
            if self.report_path is not None:
                with self.report_path.open("a", encoding="utf-8") as file:
                    file.write(f"\nFinish time: {datetime.now().isoformat(timespec='seconds')}\n")
            if self.laser is not None:
                self.laser.stop_safely()
            if self.stop_event.is_set():
                self.log("Circular scan stopped")
            else:
                self.log("Circular scan finished")
            self.log_queue.put(self.done_token)

    def return_to_radius_pose(self):
        if self.dobot is None or self.radius_pose is None:
            return

        print("Return to radius-adjusted vertical pose:", self.radius_pose)
        # MovJ names user/tool explicitly below, so only define the frame here.
        self.dobot.SetTool(config.TOOL_INDEX, config.TOOL_FRAME)
        move_result = self.dobot.dashboard.MovJ(
            *self.radius_pose,
            0,
            user=config.CIRCLE_USER_INDEX,
            tool=config.TOOL_INDEX,
            a=config.CIRCLE_ACCELERATION_RATIO,
            v=config.CIRCLE_VELOCITY_RATIO,
            cp=config.CIRCLE_CP,
        )
        print("MovJ return radius pose:", move_result)
        if not self.dobot.WaitCommandDone(move_result):
            raise RuntimeError("Return to radius-adjusted pose failed or timed out")
        self.radius_returned = True

    def request_robot_stop(self):
        self.stop_event.set()
        if self.laser is not None:
            try:
                self.laser.stop_safely()
            except Exception as error:
                self.log(f"ERROR stopping laser: {error}")
        if self.dobot is None:
            return
        try:
            stop_result = self.dobot.dashboard.Stop()
            self.log(f"Stop: {stop_result}")
        except Exception as error:
            self.log(f"ERROR stopping robot: {error}")

    def stop_scan(self):
        self.log("Stop requested")
        self.request_robot_stop()
        if not self.running:
            try:
                self.return_to_radius_pose()
            except Exception as error:
                self.log(f"ERROR: {error}")

    def _jog_ready(self):
        return (
            self.prepared
            and self.dobot is not None
            and self.center_pose is not None
            and not (self.worker and self.worker.is_alive())
        )

    def _sync_center_pose_and_poses(self, indices):
        # Re-read the settled pose, update the changed components of the circle
        # center, and regenerate the circle poses. Tk vars are touched via the
        # queue so this is safe to call from a worker thread.
        pose = circularmove.get_circle_tool_cartesian_pose(self.dobot)
        for index in indices:
            self.center_pose[index] = pose[index]
        config.CIRCLE_RZ_DEG = self.center_pose[5]
        # The new Rz becomes the saved Rz: the radius/home pose shares the same
        # orientation (tool 1 and tool 2 differ only in z), so returning later
        # comes back to this Rz too.
        if self.radius_pose is not None:
            self.radius_pose = list(self.radius_pose)
            self.radius_pose[5] = self.center_pose[5]
        self.poses = circularmove.generate_tool_center_circle_poses(
            self.center_pose,
            total_steps=config.CIRCLE_TOTAL_STEPS,
            arc_deg=config.CIRCLE_ARC_DEG,
        )
        self.queue_status("rz", round(self.center_pose[5], 2))
        self.queue_status("center", self.center_pose)
        self.queue_status("progress", f"0/{len(self.poses)}")

    def jog_rz_start(self, direction):
        # Hold-to-jog: rotate Rz continuously while the button is pressed.
        # Only allowed once initialized and while no scan/init worker is running.
        if not self._jog_ready():
            return
        try:
            # coordtype=2 (tool coordinate) + the ORIGINAL tool frame: Rz rotates
            # about the real probe's own Z axis. The circle center sits on that
            # same Z axis, so it does not move — we just re-read it on stop.
            self.dobot.dashboard.MoveJog(
                f"Rz{direction}",
                coordtype=2,
                tool=config.TOOL_INDEX,
            )
        except Exception as error:
            self.log(f"ERROR: {error}")

    def jog_rz_stop(self):
        if self.dobot is None:
            return
        try:
            self.dobot.dashboard.MoveJog()  # empty axis stops the jog
        except Exception as error:
            self.log(f"ERROR: {error}")
            return

        if not self.prepared or self.center_pose is None:
            return
        try:
            self._sync_center_pose_and_poses([5])
        except Exception as error:
            self.log(f"ERROR: {error}")

    def move_to_rz(self):
        if not self._jog_ready():
            return
        try:
            target_rz = float(self.rz_var.get())
        except (tk.TclError, ValueError):
            messagebox.showerror("Invalid Rz", "Enter a numeric Rz value")
            return

        self.set_busy(True)
        self.worker = threading.Thread(target=self._run_move_to_rz, args=(target_rz,), daemon=True)
        self.worker.start()

    def _run_move_to_rz(self, target_rz):
        try:
            with contextlib.redirect_stdout(self):
                target = list(self.center_pose)
                target[5] = target_rz
                print("Move to Rz:", target_rz)
                circularmove.run_step(
                    self.dobot,
                    target,
                    user=config.CIRCLE_USER_INDEX,
                    tool=config.CIRCLE_TOOL_INDEX,
                    acceleration=config.CIRCLE_ACCELERATION_RATIO,
                    velocity=config.CIRCLE_VELOCITY_RATIO,
                    cp=config.CIRCLE_CP,
                    circle_tool_frame=self.circle_tool_frame,
                )
                self._sync_center_pose_and_poses([5])
        except Exception as error:
            self.log(f"ERROR: {error}")
        finally:
            self.log_queue.put(self.done_token)

    def set_busy(self, busy):
        self._busy = busy
        self.init_button.configure(state="normal" if not busy and not self.devices_ready else "disabled")
        self.level_xz_button.configure(
            state="normal" if not busy and self.devices_ready else "disabled"
        )
        self.set_radius_button.configure(
            state="normal" if not busy and self.devices_ready else "disabled"
        )
        self.start_button.configure(state="normal" if self.prepared and not busy else "disabled")
        self.stop_button.configure(state="normal" if self.running or self.prepared else "disabled")
        jog_ready = self.prepared and not busy and self.dobot is not None
        jog_state = "normal" if jog_ready else "disabled"
        self.rz_minus_button.configure(state=jog_state)
        self.rz_plus_button.configure(state=jog_state)
        self.move_rz_button.configure(state=jog_state)

    def queue_status(self, kind, value):
        self.log_queue.put(("status", kind, value))

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
                item = self.log_queue.get_nowait()
            except queue.Empty:
                break

            if item is self.done_token:
                self.running = False
                self.set_busy(False)
            elif isinstance(item, tuple) and item[0] == "status":
                _, kind, value = item
                if kind == "center":
                    self.center_pose_var.set(str(value))
                elif kind == "progress":
                    self.progress_var.set(str(value))
                elif kind == "rz":
                    self.rz_var.set(value)
            else:
                self.log_text.insert("end", str(item).rstrip() + "\n")
                self.log_text.see("end")
        self.after(100, self.drain_log)

    def on_close(self):
        self.request_robot_stop()
        if self.worker and self.worker.is_alive():
            self.worker.join(timeout=120)
        if not (self.worker and self.worker.is_alive()) and not self.radius_returned:
            try:
                self.return_to_radius_pose()
            except Exception as error:
                self.log(f"ERROR: {error}")
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
