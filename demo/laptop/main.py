#!/usr/bin/env python3
"""Live IMU glider demo: standard-library Python + tkinter.

Usage:
    python main.py 192.168.1.50
    python main.py 192.168.1.50 --fullscreen

Keys:
    t   temporary hardware Z-tare, then reset the display reference
    x   stop the Pi demo application and close the viewer

The window must have keyboard focus.
"""

import argparse
import math
import queue
import select
import socket
import threading
import time
import tkinter as tk

from glider import (
    BACKGROUND,
    DEFAULT_AXES,
    IDENTITY,
    Glider,
    axis_matrix,
    determinant,
    relative_rotation,
)


TARGET_HZ = 30.0
NETWORK_STALE_SECONDS = 0.75
SENSOR_STALE_MS = 250
EXIT_TIMEOUT_SECONDS = 3.0

PHASE_STARTING = 0
PHASE_TRACKING = 1
PHASE_TARE_SETTLE = 2
PHASE_TARE_POST = 3
PHASE_RESTORING = 4
PHASE_FAULT = 5

PHASE_NAMES = {
    PHASE_STARTING: "Starting IMU",
    PHASE_TRACKING: "Tracking",
    PHASE_TARE_SETTLE: "Taring: hold reference pose",
    PHASE_TARE_POST: "Taring: waiting for fresh orientation",
    PHASE_RESTORING: "Restoring acquisition",
    PHASE_FAULT: "IMU fault",
}

ERROR_NAMES = {
    0: "",
    1: "IMU startup failed",
    2: "IMU session fault",
    3: "Could not configure tare",
    4: "Tare command failed",
    5: "No usable tare-stage orientation",
    6: "Could not restore acquisition",
    7: "No usable startup orientation",
    8: "Sensor reset detected; restart the Pi demo",
}


def set_dpi_awareness():
    try:
        import ctypes
        try:
            ctypes.windll.shcore.SetProcessDpiAwareness(1)
        except (AttributeError, OSError):
            ctypes.windll.user32.SetProcessDPIAware()
    except (ImportError, AttributeError, OSError):
        pass


def parse_snapshot(line, session):
    fields = line.split()

    if len(fields) != 17 or fields[0] != "D" or fields[1] != "1":
        raise ValueError("invalid telemetry frame")

    values = [int(value) for value in fields[2:13]]
    q = tuple(float(value) for value in fields[13:17])

    if not all(math.isfinite(value) for value in q):
        raise ValueError("non-finite quaternion")

    norm2 = sum(value * value for value in q)
    if not 0.25 <= norm2 <= 2.25:
        raise ValueError("invalid quaternion magnitude")

    (
        serial, reference, epoch, sequence, age_ms, valid, phase,
        tare_id, tare_result, error, accuracy
    ) = values

    if valid not in (0, 1) or phase not in PHASE_NAMES:
        raise ValueError("invalid telemetry state")

    if tare_result not in (-1, 0, 1, 2):
        raise ValueError("invalid tare result")

    if min(serial, reference, epoch, sequence, age_ms, tare_id) < 0:
        raise ValueError("negative telemetry identity")

    return {
        "session": session,
        "serial": serial,
        "reference": reference,
        "epoch": epoch,
        "sequence": sequence,
        "age_ms": age_ms,
        "valid": bool(valid),
        "phase": phase,
        "tare_id": tare_id,
        "tare_result": tare_result,
        "error": error,
        "accuracy": accuracy,
        "q": q,
        "received": time.perf_counter(),
    }


class Link:
    """Networking worker; never touches tkinter objects."""

    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.lock = threading.Lock()
        self.commands = queue.Queue(maxsize=4)
        self.quit_requested = threading.Event()
        self.finished = threading.Event()
        self.pi_stopped = threading.Event()

        self.latest = None
        self.connected = False
        self.message = "Connecting to Pi"
        self.rejected_id = 0
        self.thread = threading.Thread(
            target=self._run, name="demo-network", daemon=True
        )

    def start(self):
        self.thread.start()

    def state(self):
        with self.lock:
            return (
                self.latest,
                self.connected,
                self.message,
                self.rejected_id,
            )

    def _status(self, connected, message):
        with self.lock:
            self.connected = connected
            self.message = message

    def tare(self, request_id):
        if self.quit_requested.is_set():
            return False

        try:
            self.commands.put_nowait(f"T {request_id}\n".encode("ascii"))
            return True
        except queue.Full:
            return False

    def stop(self):
        self.quit_requested.set()

    def _discard_commands(self):
        while True:
            try:
                self.commands.get_nowait()
            except queue.Empty:
                return

    @staticmethod
    def _send(sock, payload):
        deadline = time.perf_counter() + 0.4
        remaining = memoryview(payload)

        while remaining:
            try:
                count = sock.send(remaining)
                if count == 0:
                    raise ConnectionError("socket closed while sending")
                remaining = remaining[count:]
            except BlockingIOError:
                if time.perf_counter() >= deadline:
                    raise TimeoutError("command send timed out")
                select.select([], [sock], [], 0.02)

    def _session(self, sock):
        buffer = bytearray()
        session = None
        exit_sent = False
        exit_deadline = None
        last_received = time.perf_counter()

        while True:
            if self.quit_requested.is_set() and not exit_sent:
                self._discard_commands()
                self._send(sock, b"X\n")
                exit_sent = True
                exit_deadline = time.perf_counter() + EXIT_TIMEOUT_SECONDS
                self._status(True, "Waiting for Pi application to stop")

            if not exit_sent:
                try:
                    payload = self.commands.get_nowait()
                except queue.Empty:
                    payload = None

                if payload is not None:
                    self._send(sock, payload)

            readable, _, exceptional = select.select(
                [sock], [], [sock], 0.02
            )

            if exceptional:
                raise ConnectionError("socket error")

            if readable:
                chunk = sock.recv(8192)

                if not chunk:
                    raise ConnectionError(
                        "Pi disconnected before stop acknowledgment"
                        if exit_sent else "Pi disconnected"
                    )

                last_received = time.perf_counter()
                buffer.extend(chunk)

                if len(buffer) > 65536:
                    raise ValueError("receive buffer limit exceeded")

                while True:
                    position = buffer.find(b"\n")
                    if position < 0:
                        break

                    line = bytes(buffer[:position]).decode("ascii")
                    del buffer[:position + 1]
                    line = line.rstrip("\r")

                    if line.startswith("H "):
                        fields = line.split()
                        if (len(fields) != 3 or fields[1] != "1"
                                or session is not None):
                            raise ValueError("unsupported server handshake")
                        int(fields[2], 16)
                        session = fields[2]

                    elif line.startswith("D "):
                        if session is None:
                            raise ValueError("telemetry before handshake")

                        snapshot = parse_snapshot(line, session)

                        with self.lock:
                            self.latest = snapshot
                            self.connected = True
                            if not exit_sent:
                                self.message = "Connected"

                    elif line.startswith("E "):
                        fields = line.split()
                        if len(fields) != 2:
                            raise ValueError("invalid rejection message")
                        rejected = int(fields[1])
                        with self.lock:
                            self.rejected_id = rejected

                    elif line == "BYE":
                        self.pi_stopped.set()
                        self._status(False, "Pi demo application stopped")
                        return

                    else:
                        raise ValueError("unknown server message")

            now = time.perf_counter()

            if exit_sent:
                if now >= exit_deadline:
                    raise TimeoutError("Pi stop acknowledgment timed out")
            elif now - last_received > 2.0:
                raise TimeoutError("no telemetry for two seconds")

    def _run(self):
        try:
            while not self.quit_requested.is_set():
                sock = None

                try:
                    self._status(False, f"Connecting to {self.host}:{self.port}")
                    sock = socket.create_connection(
                        (self.host, self.port), timeout=1.0
                    )
                    sock.setsockopt(
                        socket.IPPROTO_TCP, socket.TCP_NODELAY, 1
                    )
                    sock.setblocking(False)
                    self._status(True, "Connected; waiting for telemetry")
                    self._session(sock)

                    if self.pi_stopped.is_set():
                        break

                except (OSError, ValueError) as exc:
                    self._status(False, str(exc))

                finally:
                    self._discard_commands()
                    if sock is not None:
                        sock.close()

                if self.quit_requested.is_set():
                    break

                self.quit_requested.wait(0.75)

        finally:
            self.finished.set()


class Viewer:
    def __init__(self, args):
        self.args = args
        self.mapping = axis_matrix(args.axes)
        self.model = Glider(args.azimuth, args.elevation)
        self.link = Link(args.host, args.port)

        self.reference_q = None
        self.reference_key = None
        self.rotation = IDENTITY
        self.last_identity = None
        self.pending_tare = None
        self.next_request = 0

        self.notice = ""
        self.notice_until = 0.0
        self.t_down = False
        self.closing = False
        self.close_deadline = 0.0
        self.destroyed = False

        self.frames = 0
        self.fps = 0.0
        self.fps_start = time.perf_counter()
        self.next_frame = self.fps_start

        self.root = tk.Tk()
        self.root.title("Glider IMU orientation demo")

        if args.fullscreen:
            self.root.attributes("-fullscreen", True)
        else:
            self.root.geometry("1100x800")
            self.root.minsize(640, 480)

        self.canvas = tk.Canvas(
            self.root, bg=BACKGROUND, highlightthickness=0
        )
        self.canvas.pack(fill="both", expand=True)

        self.root.bind("<KeyPress>", self.on_key_press)
        self.root.bind("<KeyRelease>", self.on_key_release)
        self.root.protocol("WM_DELETE_WINDOW", self.begin_exit)
        self.root.after(100, self.canvas.focus_set)

        if determinant(self.mapping) < 0:
            print(
                "WARNING: --axes uses a reflected coordinate mapping. "
                "The supplied forward/left/down description is not a "
                "right-handed physical sensor frame. Verify rotation signs."
            )

    def flash(self, text, seconds=3.0):
        self.notice = text
        self.notice_until = time.perf_counter() + seconds

    def on_key_press(self, event):
        key = (event.keysym or "").lower()

        if key == "x":
            self.begin_exit()
            return "break"

        if key == "t":
            if not self.t_down:
                self.t_down = True
                self.request_tare()
            return "break"

        return None

    def on_key_release(self, event):
        if (event.keysym or "").lower() == "t":
            self.t_down = False

    def request_tare(self):
        if self.closing or self.pending_tare is not None:
            return

        data, connected, _, _ = self.link.state()
        now = time.perf_counter()

        if (
            not connected
            or data is None
            or now - data["received"] > NETWORK_STALE_SECONDS
            or not data["valid"]
            or data["age_ms"] > SENSOR_STALE_MS
            or data["phase"] != PHASE_TRACKING
            or self.reference_q is None
        ):
            self.flash("Tare unavailable: wait for fresh tracking data")
            return

        self.next_request = (
            max(self.next_request, data["tare_id"]) + 1
        ) & 0xFFFFFFFF

        if self.next_request == 0:
            self.next_request = 1

        request_id = self.next_request

        if self.link.tare(request_id):
            self.pending_tare = (request_id, now)
            self.flash("Taring: hold the glider in the reference pose", 8.0)
        else:
            self.flash("Could not queue tare request")

    def begin_exit(self):
        if self.closing:
            return

        self.closing = True
        self.close_deadline = (
            time.perf_counter() + EXIT_TIMEOUT_SECONDS + 0.5
        )
        self.link.stop()

    def destroy(self):
        if self.destroyed:
            return

        self.destroyed = True
        self.root.destroy()

    def update_pose(self, data, connected, rejected_id, now):
        if self.pending_tare is not None:
            request_id, started = self.pending_tare

            if rejected_id == request_id:
                self.pending_tare = None
                self.flash("Tare rejected: Pi was busy")
            elif data is not None and data["tare_id"] == request_id:
                if data["tare_result"] == 2:
                    self.pending_tare = None
                    self.flash("Reference reset")
                elif data["tare_result"] == -1:
                    self.pending_tare = None
                    self.flash(
                        "Tare warning: " +
                        ERROR_NAMES.get(data["error"], "operation failed"),
                        5.0,
                    )
            elif now - started > 10.0:
                self.pending_tare = None
                self.flash(
                    "No tare completion received; check connection/status",
                    5.0,
                )

        if (
            not connected
            or data is None
            or now - data["received"] > NETWORK_STALE_SECONDS
            or not data["valid"]
            or data["age_ms"] > SENSOR_STALE_MS
            or data["phase"] != PHASE_TRACKING
            or self.pending_tare is not None
        ):
            return

        key = (data["session"], data["reference"])
        identity = (
            data["session"], data["epoch"], data["sequence"]
        )

        if key != self.reference_key:
            self.reference_q = data["q"]
            self.reference_key = key
            self.rotation = IDENTITY
            self.last_identity = None

        if identity != self.last_identity:
            self.rotation = relative_rotation(
                self.reference_q,
                data["q"],
                self.mapping,
                inverse=self.args.inverse_quaternion,
            )
            self.last_identity = identity

    def status(self, data, connected, message, now):
        if self.closing:
            return "Stopping demo application...", "#b06a00"

        if not connected:
            return f"Disconnected — {message}", "#b42318"

        if data is None:
            return "Connected — waiting for IMU data", "#b06a00"

        if now - data["received"] > NETWORK_STALE_SECONDS:
            return "Telemetry stale — holding last pose", "#b42318"

        phase = data["phase"]

        if phase == PHASE_FAULT:
            reason = ERROR_NAMES.get(data["error"], "unknown error")
            return f"IMU fault — {reason}", "#b42318"

        if self.pending_tare is not None:
            return "Taring — hold reference pose", "#b06a00"

        if phase != PHASE_TRACKING:
            return PHASE_NAMES.get(phase, "Working"), "#b06a00"

        if not data["valid"] or data["age_ms"] > SENSOR_STALE_MS:
            return "IMU data stale — holding last pose", "#b42318"

        return "Tracking", "#18723b"

    def tick(self):
        if self.destroyed:
            return

        now = time.perf_counter()
        data, connected, message, rejected_id = self.link.state()

        if self.closing:
            if self.link.finished.is_set() or now >= self.close_deadline:
                if self.link.pi_stopped.is_set():
                    print("Pi demo stopped. Raspberry Pi remains running.")
                else:
                    print(
                        "WARNING: Pi stop was not acknowledged. "
                        "The backend may still be running; use Ctrl+C "
                        "in its terminal if necessary."
                    )
                self.destroy()
                return

        try:
            if not self.closing:
                self.update_pose(data, connected, rejected_id, now)
        except ValueError as exc:
            self.flash(f"Orientation rejected: {exc}")

        width = max(320, self.canvas.winfo_width())
        height = max(240, self.canvas.winfo_height())

        self.canvas.delete("all")
        self.model.draw(self.canvas, width, height, self.rotation)

        title, color = self.status(data, connected, message, now)

        self.canvas.create_text(
            20, 18, anchor="nw",
            text="GLIDER ORIENTATION",
            fill="#263244", font=("Segoe UI", 18, "bold"),
        )
        self.canvas.create_text(
            20, 54, anchor="nw",
            text=title, fill=color, font=("Segoe UI", 13, "bold"),
        )

        detail = f"Viewer {self.fps:.1f} fps"

        if data is not None:
            detail += (
                f"   |   RV status {data['accuracy']}/3"
                f"   |   Pi sample age {data['age_ms']} ms"
            )

        self.canvas.create_text(
            20, 82, anchor="nw",
            text=detail, fill="#596579", font=("Segoe UI", 10),
        )

        self.canvas.create_text(
            20, height - 45, anchor="sw",
            text="Reference: nose LEFT, top UP. Model rotates in place.",
            fill="#596579", font=("Segoe UI", 11),
        )
        self.canvas.create_text(
            20, height - 18, anchor="sw",
            text="t: temporary Z-tare + reference reset     x: exit demo"
                 "     Click this window for keyboard focus",
            fill="#38465a", font=("Segoe UI", 10),
        )

        if now < self.notice_until and not self.closing:
            self.canvas.create_text(
                width / 2, 125, anchor="n",
                width=max(280, width - 80),
                text=self.notice, fill="#a45e00",
                font=("Segoe UI", 13, "bold"),
            )

        self.frames += 1
        done = time.perf_counter()

        if done - self.fps_start >= 1.0:
            self.fps = self.frames / (done - self.fps_start)
            self.frames = 0
            self.fps_start = done

        # Skip missed deadlines instead of scheduling catch-up redraws.
        period = 1.0 / TARGET_HZ
        self.next_frame += period

        if self.next_frame < done:
            self.next_frame = done

        delay_ms = max(
            1, math.ceil((self.next_frame - time.perf_counter()) * 1000)
        )
        self.root.after(delay_ms, self.tick)

    def run(self):
        self.link.start()
        self.root.after(10, self.tick)

        try:
            self.root.mainloop()
        finally:
            self.link.stop()
            self.link.thread.join(timeout=EXIT_TIMEOUT_SECONDS + 0.5)


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("host", help="Raspberry Pi IPv4 address or hostname")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--fullscreen", action="store_true")
    parser.add_argument(
        "--axes", default=DEFAULT_AXES,
        help="model X/Y/Z as signed sensor axes; default: %(default)s. "
             "Use --axes=-x,-y,-z when the value begins with '-'.",
    )
    parser.add_argument("--azimuth", type=float, default=20.0)
    parser.add_argument("--elevation", type=float, default=25.0)
    parser.add_argument(
        "--inverse-quaternion", action="store_true",
        help="transpose incoming quaternion rotations; diagnostic option "
             "for validating the sensor's rotation convention",
    )
    args = parser.parse_args()

    if not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")

    if not math.isfinite(args.azimuth) or not math.isfinite(args.elevation):
        parser.error("view angles must be finite")

    try:
        axis_matrix(args.axes)
    except ValueError as exc:
        parser.error(str(exc))

    return args


def main():
    args = arguments()
    set_dpi_awareness()

    try:
        Viewer(args).run()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    