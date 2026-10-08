#!/usr/bin/env python3
"""
Teensy Bode UI - desktop app for the Teensy 4.1 transducer impedance analyzer.

Works with the TeensyBodeUI.ino firmware (text protocol over USB serial).

Install once:   pip install pyserial matplotlib numpy
Run:            python teensy_bode_ui.py

Close the Arduino Serial Monitor / Plotter first - only one program can use the port.
"""

import csv
import math
import queue
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure

TEENSY_VID = 0x16C0


# =====================================================================
#  Serial link
# =====================================================================
class TeensyLink:
    """Line-based request/response link to the Teensy firmware."""

    def __init__(self):
        self.ser = None
        self.lock = threading.Lock()
        self._buf = b""

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port):
        self.close()
        self.ser = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self._buf = b""
        reply = self.request("PING", ("OK",), timeout=2.0)
        if not reply or "TEENSYBODE" not in reply[-1]:
            self.close()
            raise IOError("Device did not answer PING - is TeensyBodeUI.ino uploaded?")
        return reply[-1]

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write(self, text):
        if self.connected:
            self.ser.write((text + "\n").encode())

    def readline(self, timeout):
        """Return one decoded line, or None on timeout."""
        t_end = time.time() + timeout
        while True:
            if b"\n" in self._buf:
                line, self._buf = self._buf.split(b"\n", 1)
                return line.decode(errors="replace").strip()
            if time.time() > t_end:
                return None
            chunk = self.ser.read(self.ser.in_waiting or 1)
            if chunk:
                self._buf += chunk

    def request(self, cmd, terminators, line_cb=None, timeout=5.0, stop_flag=None):
        """
        Send cmd, collect lines until one starts with any terminator prefix (or ERR).
        'timeout' is the maximum gap between lines. line_cb(line) is called for every line.
        Returns the list of lines (including the terminator), or None on timeout/abort.
        """
        with self.lock:
            self.write(cmd)
            lines = []
            stop_sent = False
            while True:
                if stop_flag is not None and stop_flag.is_set() and not stop_sent:
                    self.write("STOP")
                    stop_sent = True
                line = self.readline(timeout)
                if line is None:
                    return None
                if not line:
                    continue
                if line.startswith("OK STOP"):
                    if stop_sent:
                        return None
                    continue
                lines.append(line)
                if line_cb:
                    line_cb(line)
                if line.startswith("ERR"):
                    return lines
                if line.startswith("SABORT"):
                    # firmware stopped the sweep; swallow its "OK STOP" reply
                    self.readline(1.0)
                    return None
                if any(line.startswith(t) for t in terminators):
                    return lines


# =====================================================================
#  Analysis
# =====================================================================
def zero_crossings(f, ph, rising):
    """Linear-interpolated phase zero crossings. rising=True: - to + (fs)."""
    out = []
    for i in range(len(f) - 1):
        a, b = ph[i], ph[i + 1]
        if np.isnan(a) or np.isnan(b):
            continue
        if (rising and a < 0 <= b) or (not rising and a >= 0 > b):
            out.append(f[i] + (f[i + 1] - f[i]) * (0 - a) / (b - a))
    return out


def analyze(fine, coarse):
    """fine/coarse: lists of (f, zmag, phase, re, im). Returns a dict of results."""
    r = {}
    data = fine if fine else coarse
    if not data:
        return r
    arr = np.array(sorted(data), dtype=float)
    f, z, ph = arr[:, 0], arr[:, 1], arr[:, 2]
    ok = ~np.isnan(z)
    f, z, ph = f[ok], z[ok], ph[ok]
    if len(f) < 3:
        return r

    fs_list = zero_crossings(f, ph, rising=True)
    fs = fs_list[0] if fs_list else None
    fp = None
    if fs is not None:
        fp_list = [x for x in zero_crossings(f, ph, rising=False) if x > fs]
        fp = fp_list[0] if fp_list else None
    r["fs"], r["fp"] = fs, fp

    imin, imax, ipm = int(np.argmin(z)), int(np.argmax(z)), int(np.argmax(ph))
    r["zmin"], r["f_zmin"] = z[imin], f[imin]
    r["zmax"], r["f_zmax"] = z[imax], f[imax]
    r["maxphase"], r["f_maxphase"] = ph[ipm], f[ipm]

    # C0 from the lowest-frequency coarse point (capacitive region)
    src = coarse if coarse else data
    c0 = None
    if src:
        p = min(src, key=lambda t: t[0])
        if p[4] is not None and not math.isnan(p[4]) and p[4] < 0:
            c0 = -1.0 / (2 * math.pi * p[0] * p[4])
    r["C0"] = c0

    if fs and fp and fp > fs:
        r["df"] = fp - fs
        r["keff"] = math.sqrt(1 - (fs / fp) ** 2)
        if c0:
            c1 = c0 * ((fp / fs) ** 2 - 1)
            qm = 1.0 / (2 * math.pi * fs * c1 * r["zmin"])
            r["Qm"] = qm
            r["M"] = qm * c1 / c0
    return r


def fmt_ohm(v):
    if v is None or (isinstance(v, float) and math.isnan(v)):
        return "-"
    if v >= 1e6:
        return f"{v/1e6:.3f} MΩ"
    if v >= 1e3:
        return f"{v/1e3:.3f} kΩ"
    return f"{v:.2f} Ω"


def fmt_hz(v):
    return "-" if v is None else f"{v:,.1f} Hz"


# =====================================================================
#  GUI
# =====================================================================
class App:
    def __init__(self, root):
        self.root = root
        root.title("Teensy Bode - Transducer Impedance Analyzer")
        root.geometry("1300x860")

        self.link = TeensyLink()
        self.events = queue.Queue()
        self.stop_flag = threading.Event()
        self.busy = False
        self.live_wave = tk.BooleanVar(value=False)

        self.coarse = []      # (f, |Z|, phase, re, im)
        self.fine = []
        self.reference = None
        self.results = {}

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self._poll_events)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- layout ----------------
    def _build_ui(self):
        left = ttk.Frame(self.root, padding=6)
        left.pack(side=tk.LEFT, fill=tk.Y)
        right = ttk.Frame(self.root, padding=(0, 6, 6, 6))
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- connection ---
        box = ttk.LabelFrame(left, text="Connection", padding=6)
        box.pack(fill=tk.X, pady=(0, 6))
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(box, textvariable=self.port_var, width=28, state="readonly")
        self.port_combo.grid(row=0, column=0, columnspan=2, sticky="ew")
        ttk.Button(box, text="Refresh", command=self.refresh_ports).grid(row=1, column=0, sticky="ew", pady=2)
        self.connect_btn = ttk.Button(box, text="Connect", command=self.toggle_connect)
        self.connect_btn.grid(row=1, column=1, sticky="ew", pady=2)
        self.conn_label = ttk.Label(box, text="Not connected", foreground="#a33")
        self.conn_label.grid(row=2, column=0, columnspan=2, sticky="w")

        # --- setup / calibration ---
        box = ttk.LabelFrame(left, text="Setup & calibration", padding=6)
        box.pack(fill=tk.X, pady=(0, 6))
        ttk.Label(box, text="Sense resistor RS (Ω)").grid(row=0, column=0, sticky="w")
        self.rs_var = tk.StringVar(value="1000")
        ttk.Entry(box, textvariable=self.rs_var, width=9).grid(row=0, column=1)
        ttk.Button(box, text="Apply", command=self.apply_rs).grid(row=0, column=2, padx=(4, 0))
        ttk.Button(box, text="Open-circuit cal (DUT removed)", command=self.open_cal).grid(
            row=1, column=0, columnspan=3, sticky="ew", pady=(4, 2))
        ttk.Button(box, text="Clear calibration", command=self.clear_cal).grid(
            row=2, column=0, columnspan=3, sticky="ew")
        self.cal_label = ttk.Label(box, text="Calibration: built-in R4/C4 model")
        self.cal_label.grid(row=3, column=0, columnspan=3, sticky="w", pady=(2, 0))

        # --- sweep settings ---
        box = ttk.LabelFrame(left, text="Sweep settings", padding=6)
        box.pack(fill=tk.X, pady=(0, 6))
        self.params = {}
        rows = [
            ("start", "Start (Hz)", "20000"),
            ("stop", "Stop (Hz)", "40000"),
            ("step", "Coarse step (Hz)", "50"),
            ("c_settle", "Coarse settle (ms)", "20"),
            ("c_meas", "Coarse measure (ms)", "20"),
            ("span", "Zoom span (Hz)", "800"),
            ("npts", "Zoom points", "100"),
            ("f_settle", "Zoom settle (ms)", "60"),
            ("f_meas", "Zoom measure (ms)", "40"),
        ]
        for i, (key, label, default) in enumerate(rows):
            ttk.Label(box, text=label).grid(row=i, column=0, sticky="w")
            v = tk.StringVar(value=default)
            ttk.Entry(box, textvariable=v, width=10).grid(row=i, column=1, sticky="e")
            self.params[key] = v
        self.refine_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="Refine fs / fp (fine steps)", variable=self.refine_var).grid(
            row=len(rows), column=0, columnspan=2, sticky="w", pady=(4, 0))

        btns = ttk.Frame(left)
        btns.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(btns, text="Find resonance (auto)", command=self.run_auto).pack(fill=tk.X)
        ttk.Button(btns, text="Coarse sweep only", command=self.run_coarse).pack(fill=tk.X, pady=2)
        ttk.Button(btns, text="STOP", command=self.stop).pack(fill=tk.X)
        f2 = ttk.Frame(btns)
        f2.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(f2, text="Save CSV", command=self.save_csv).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(f2, text="Load reference", command=self.load_reference).pack(side=tk.LEFT, expand=True, fill=tk.X)
        ttk.Button(btns, text="Clear reference", command=self.clear_reference).pack(fill=tk.X, pady=2)

        # --- bottom strip: results + log ---
        bottom = ttk.Frame(right)
        bottom.pack(side=tk.BOTTOM, fill=tk.X, pady=(6, 0))
        self.status = ttk.Label(right, text="Ready", anchor="w")
        self.status.pack(side=tk.BOTTOM, fill=tk.X)
        box = ttk.LabelFrame(bottom, text="Results", padding=(8, 2))
        box.pack(side=tk.LEFT, fill=tk.Y)
        self.res_vars = {}
        res_rows = [("fs", "fs (series)"), ("fp", "fp (parallel)"), ("df", "fp - fs"),
                    ("zmin", "|Z| min"), ("zmax", "|Z| max"), ("maxphase", "Max phase"),
                    ("C0", "C0"), ("keff", "k_eff"), ("Qm", "Qm (approx)"), ("M", "M = Qm·C1/C0")]
        for i, (key, label) in enumerate(res_rows):
            r, c = i % 5, (i // 5) * 2
            ttk.Label(box, text=label).grid(row=r, column=c, sticky="w", padx=(0 if c == 0 else 18, 0))
            v = tk.StringVar(value="-")
            ttk.Label(box, textvariable=v, font=("TkFixedFont", 10, "bold"), width=19, anchor="e").grid(
                row=r, column=c + 1, sticky="e", padx=(8, 0))
            self.res_vars[key] = v
        logcol = ttk.Frame(bottom)
        logcol.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(6, 0))
        self.log = tk.Text(logcol, height=5, wrap="word", font=("TkFixedFont", 9))
        self.log.pack(side=tk.TOP, fill=tk.BOTH, expand=True)
        cmdrow = ttk.Frame(logcol)
        cmdrow.pack(side=tk.TOP, fill=tk.X, pady=(2, 0))
        ttk.Label(cmdrow, text="Command:").pack(side=tk.LEFT)
        self.cmd_var = tk.StringVar()
        ent = ttk.Entry(cmdrow, textvariable=self.cmd_var)
        ent.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        ent.bind("<Return>", lambda e: self.send_raw())
        ttk.Button(cmdrow, text="Send", command=self.send_raw).pack(side=tk.LEFT)

        # --- right side: tabs ---
        self.tabs = ttk.Notebook(right)
        self.tabs.pack(fill=tk.BOTH, expand=True)

        # Bode tab
        tab = ttk.Frame(self.tabs)
        self.tabs.add(tab, text="Bode plot")
        self.fig = Figure(figsize=(8, 6), dpi=100)
        self.ax_z = self.fig.add_subplot(211)
        self.ax_p = self.fig.add_subplot(212, sharex=self.ax_z)
        self.fig.subplots_adjust(left=0.09, right=0.98, top=0.94, bottom=0.08, hspace=0.12)
        self.canvas = FigureCanvasTkAgg(self.fig, master=tab)
        NavigationToolbar2Tk(self.canvas, tab).update()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        zb = ttk.Frame(tab)
        zb.pack(fill=tk.X)
        ttk.Button(zb, text="Show full sweep", command=lambda: self.redraw_bode(view="full")).pack(side=tk.LEFT)
        ttk.Button(zb, text="Zoom to resonance", command=lambda: self.redraw_bode(view="zoom")).pack(side=tk.LEFT, padx=4)

        # Waveform tab
        tab = ttk.Frame(self.tabs)
        self.tabs.add(tab, text="Waveforms / single frequency")
        ctrl = ttk.Frame(tab, padding=4)
        ctrl.pack(fill=tk.X)
        ttk.Label(ctrl, text="Frequency (Hz)").pack(side=tk.LEFT)
        self.wave_f = tk.StringVar(value="30000")
        ttk.Entry(ctrl, textvariable=self.wave_f, width=10).pack(side=tk.LEFT, padx=4)
        ttk.Button(ctrl, text="Capture", command=self.capture_wave).pack(side=tk.LEFT)
        ttk.Checkbutton(ctrl, text="Live", variable=self.live_wave, command=self.capture_wave).pack(side=tk.LEFT, padx=8)
        ttk.Button(ctrl, text="At fs", command=lambda: self._wave_at("fs")).pack(side=tk.LEFT)
        ttk.Button(ctrl, text="At fp", command=lambda: self._wave_at("fp")).pack(side=tk.LEFT, padx=4)

        body = ttk.Frame(tab)
        body.pack(fill=tk.BOTH, expand=True)
        self.wfig = Figure(figsize=(7, 5), dpi=100)
        self.wax = self.wfig.add_subplot(111)
        self.wfig.subplots_adjust(left=0.1, right=0.98, top=0.94, bottom=0.1)
        vals = ttk.LabelFrame(body, text="Measured values", padding=8)
        vals.pack(side=tk.RIGHT, fill=tk.Y, padx=6, pady=6)
        self.wcanvas = FigureCanvasTkAgg(self.wfig, master=body)
        self.wcanvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.wvals = {}
        for i, (key, label) in enumerate([("f", "Frequency"), ("vppA", "Node A Vpp"), ("vppB", "Node B Vpp"),
                                          ("dcA", "Node A DC"), ("dcB", "Node B DC"), ("rawA", "A raw min/max"),
                                          ("rawB", "B raw min/max"), ("z", "|Z|"), ("ph", "Phase"),
                                          ("n", "Samples"), ("clip", "Clipping")]):
            ttk.Label(vals, text=label).grid(row=i, column=0, sticky="w", pady=1)
            v = tk.StringVar(value="-")
            ttk.Label(vals, textvariable=v, font=("TkFixedFont", 10, "bold"), width=18, anchor="e").grid(
                row=i, column=1, sticky="e", padx=(10, 0))
            self.wvals[key] = v


        self.redraw_bode()
        self._draw_wave_empty()

    # ---------------- helpers ----------------
    def say(self, text):
        self.log.insert(tk.END, time.strftime("%H:%M:%S  ") + text + "\n")
        self.log.see(tk.END)

    def p(self, key, cast=float):
        return cast(float(self.params[key].get()))

    def _need_link(self):
        if not self.link.connected:
            messagebox.showwarning("Not connected", "Connect to the Teensy first.")
            return False
        if self.busy:
            self.say("Busy - press STOP first.")
            return False
        return True

    def _start_job(self, target, *args):
        self.busy = True
        self.stop_flag.clear()
        threading.Thread(target=self._job_wrapper, args=(target,) + args, daemon=True).start()

    def _job_wrapper(self, target, *args):
        try:
            target(*args)
        except Exception as e:  # report any error in the GUI
            self.events.put(("log", f"Error: {e}"))
        finally:
            self.events.put(("done", None))

    # ---------------- connection ----------------
    def refresh_ports(self):
        ports = list(serial.tools.list_ports.comports())
        items = []
        teensy_idx = None
        for i, p in enumerate(ports):
            tag = " (Teensy)" if p.vid == TEENSY_VID else ""
            items.append(f"{p.device}{tag} - {p.description}")
            if tag and teensy_idx is None:
                teensy_idx = i
        self.port_combo["values"] = items
        if items:
            self.port_combo.current(teensy_idx if teensy_idx is not None else 0)

    def toggle_connect(self):
        if self.link.connected:
            self.stop()
            self.link.close()
            self.conn_label.config(text="Not connected", foreground="#a33")
            self.connect_btn.config(text="Connect")
            self.say("Disconnected.")
            return
        sel = self.port_var.get()
        if not sel:
            messagebox.showwarning("No port", "No serial port selected.")
            return
        port = sel.split(" ")[0]
        try:
            reply = self.link.open(port)
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            return
        self.conn_label.config(text=f"Connected: {port}", foreground="#262")
        self.connect_btn.config(text="Disconnect")
        self.say(f"Connected to {port}: {reply}")
        self.apply_rs()

    def on_close(self):
        self.stop()
        time.sleep(0.2)
        self.link.close()
        self.root.destroy()

    # ---------------- simple commands ----------------
    def apply_rs(self):
        if not self.link.connected or self.busy:
            return
        try:
            rs = float(self.rs_var.get())
        except ValueError:
            messagebox.showerror("RS", "RS must be a number.")
            return
        r = self.link.request(f"RS {rs}", ("OK",), timeout=2)
        self.say(r[-1] if r else "No reply to RS")

    def open_cal(self):
        if not self._need_link():
            return
        if not messagebox.askokcancel("Open-circuit calibration",
                                      "Disconnect the transducer (leave everything else connected), then press OK."):
            return
        def job():
            dbg = []
            # 3 freq x (100 ms settle + 300 ms meas) + margin ~= 1.5 s
            r = self.link.request("OPENCAL", ("CAL ", "ERR"), timeout=5,
                                  line_cb=lambda line: dbg.append(line) if line.startswith("OCDBG") else None)
            if r and r[-1].startswith("CAL "):
                c_pf = float(r[-1].split()[1])
                if math.isnan(c_pf) or c_pf <= 0 or c_pf > 1000:
                    self.events.put(("log", f"Calibration rejected (implausible {c_pf:.1f} pF). " +
                                     (" | ".join(dbg) if dbg else "no debug lines")))
                else:
                    self.events.put(("cal", f"Calibration: open-circuit ({c_pf:.1f} pF shunt)"))
                    self.events.put(("log", f"Open-circuit calibration stored: {c_pf:.1f} pF at node B. Reconnect the DUT."))
            elif r and r[-1].startswith("ERR"):
                self.events.put(("log", f"Calibration failed: {r[-1]}. " +
                                 (" | ".join(dbg) if dbg else "")))
            else:
                self.events.put(("log", "Calibration failed / no reply (timeout - check USB CDC link). " +
                                 (" | ".join(dbg) if dbg else "no debug lines captured")))
        self._start_job(job)

    def clear_cal(self):
        if not self._need_link():
            return
        r = self.link.request("CLEARCAL", ("OK",), timeout=2)
        self.cal_label.config(text="Calibration: built-in R4/C4 model")
        self.say(r[-1] if r else "No reply")

    def send_raw(self):
        """Send a one-line settings command (e.g. DITHER 0, DDSTICKS 60) and log the reply."""
        cmd = self.cmd_var.get().strip()
        if not cmd:
            return
        if cmd.split()[0].upper() in ("SWEEP", "WAVE"):
            self.say("Use the buttons for sweeps and waveforms.")
            return
        if not self._need_link():
            return
        r = self.link.request(cmd, ("",), timeout=3)      # first reply line ends it
        self.say(f"> {cmd}   ->   {r[-1] if r else 'no reply'}")
        self.cmd_var.set("")

    def stop(self):
        self.live_wave.set(False)
        if self.busy:
            self.stop_flag.set()
            self.say("Stopping...")

    # ---------------- sweeps ----------------
    def _sweep(self, f0, f1, step, settle, meas, tag):
        """Runs one firmware sweep; streams points into the GUI. Returns list of points."""
        pts = []
        n = int((f1 - f0) / step + 1.5)
        gap = (settle + meas) / 1000.0 + 3.0

        def on_line(line):
            if line.startswith("P "):
                parts = line.split()
                try:
                    vals = tuple(float(x) for x in parts[1:6])
                except ValueError:
                    return
                pts.append(vals)
                self.events.put(("point", (tag, vals, len(pts), n)))

        r = self.link.request(f"SWEEP {f0:.3f} {f1:.3f} {step:.4f} {int(settle)} {int(meas)}",
                              ("SEND",), line_cb=on_line, timeout=gap, stop_flag=self.stop_flag)
        if r is None:
            return None
        if r and r[-1].startswith("ERR"):
            self.events.put(("log", r[-1]))
            return None
        return pts

    def run_coarse(self):
        if not self._need_link():
            return
        try:
            args = (self.p("start"), self.p("stop"), self.p("step"), self.p("c_settle"), self.p("c_meas"))
        except ValueError:
            messagebox.showerror("Settings", "All sweep settings must be numbers.")
            return
        self.coarse, self.fine = [], []
        self.redraw_bode()

        def job():
            self.events.put(("status", "Coarse sweep..."))
            pts = self._sweep(*args, tag="coarse")
            if pts is None:
                self.events.put(("log", "Sweep stopped."))
                return
            self.events.put(("analyze", "full"))
            self.events.put(("log", f"Coarse sweep done ({len(pts)} points)."))
        self._start_job(job)

    def run_auto(self):
        if not self._need_link():
            return
        try:
            start, stop, step = self.p("start"), self.p("stop"), self.p("step")
            cs, cm = self.p("c_settle"), self.p("c_meas")
            span, npts = self.p("span"), self.p("npts", int)
            fs_, fm_ = self.p("f_settle"), self.p("f_meas")
        except ValueError:
            messagebox.showerror("Settings", "All sweep settings must be numbers.")
            return
        refine = self.refine_var.get()
        self.coarse, self.fine = [], []
        self.results = {}
        self.redraw_bode()

        def job():
            t0 = time.time()
            self.events.put(("status", "1/3  Coarse sweep..."))
            coarse = self._sweep(start, stop, step, cs, cm, tag="coarse")
            if coarse is None:
                self.events.put(("log", "Stopped.")); return
            valid = [p for p in coarse if not math.isnan(p[2])]
            if not valid:
                self.events.put(("log", "No valid data in coarse sweep.")); return
            fc = max(valid, key=lambda p: p[2])[0]          # most inductive point
            self.events.put(("log", f"Coarse: max phase at {fc:,.0f} Hz - zooming in."))

            self.events.put(("status", "2/3  Zoom sweep..."))
            fstep = span / max(npts - 1, 1)
            fine = self._sweep(fc - span / 2, fc + span / 2, fstep, fs_, fm_, tag="fine")
            if fine is None:
                self.events.put(("log", "Stopped.")); return

            if refine:
                self.events.put(("status", "3/3  Refining..."))
                r = analyze(fine, coarse)
                targets = [x for x in (r.get("fs"), r.get("fp")) if x]
                if not targets:                               # no zero crossing: refine |Z| min/max
                    targets = [r.get("f_zmin"), r.get("f_zmax")]
                rstep = max(0.5, fstep / 10)
                for x in targets:
                    if x is None:
                        continue
                    pts = self._sweep(x - 2 * fstep, x + 2 * fstep, rstep, fs_, fm_, tag="fine")
                    if pts is None:
                        self.events.put(("log", "Stopped.")); return
            self.events.put(("analyze", "zoom"))
            self.events.put(("log", f"Resonance search done in {time.time() - t0:.0f} s."))
        self._start_job(job)

    # ---------------- waveform ----------------
    def _wave_at(self, key):
        v = self.results.get(key)
        if not v:
            self.say(f"No {key} available yet - run 'Find resonance' first.")
            return
        self.wave_f.set(f"{v:.1f}")
        self.capture_wave()

    def capture_wave(self):
        if not self.link.connected:
            if self.live_wave.get():
                self.live_wave.set(False)
            messagebox.showwarning("Not connected", "Connect to the Teensy first.")
            return
        if self.busy:
            return
        try:
            f = float(self.wave_f.get())
        except ValueError:
            messagebox.showerror("Frequency", "Frequency must be a number.")
            return

        def job():
            lines = self.link.request(f"WAVE {f:.3f} 100", ("WEND",), timeout=3)
            if not lines or lines[-1].startswith("ERR"):
                self.events.put(("log", "Waveform capture failed."))
                return
            head = lines[0].split()
            data = []
            for ln in lines[1:]:
                if ln.startswith("D "):
                    _, a, b = ln.split()
                    data.append((float(a), float(b)))
            self.events.put(("wave", (head, data)))
        self._start_job(job)

    def _draw_wave_empty(self):
        self.wax.clear()
        self.wax.set_xlabel("Phase (degrees, 2 cycles)")
        self.wax.set_ylabel("Volts")
        self.wax.grid(True, alpha=0.3)
        self.wcanvas.draw_idle()

    def show_wave(self, head, data):
        # head: W f |Z| ph re im vppA vppB dcA dcB minA maxA minB maxB n
        f, z, ph = float(head[1]), float(head[2]), float(head[3])
        vppA, vppB, dcA, dcB = (float(x) for x in head[6:10])
        minA, maxA, minB, maxB, n = (int(x) for x in head[10:15])
        a = np.array([d[0] for d in data] * 2)
        b = np.array([d[1] for d in data] * 2)
        x = np.arange(len(a)) * 360.0 / len(data)
        self.wax.clear()
        self.wax.plot(x, a, label="Node A", color="#1f77b4")
        self.wax.plot(x, b, label="Node B (DUT)", color="#d62728")
        self.wax.plot(x, a - b, label="A − B (∝ current)", color="#2ca02c")
        self.wax.axhline(0, color="k", lw=0.5)
        self.wax.set_xlim(0, 720)
        self.wax.set_ylim(min(-0.2, float(np.nanmin(a - b)) - 0.2), 3.5)
        self.wax.set_xlabel("Phase (degrees, 2 cycles)")
        self.wax.set_ylabel("Volts")
        self.wax.set_title(f"{f:,.1f} Hz   |Z| = {fmt_ohm(z)}   phase = {ph:+.2f}°")
        self.wax.grid(True, alpha=0.3)
        self.wax.legend(loc="upper right", fontsize=9)
        self.wcanvas.draw_idle()

        clip = minA <= 5 or minB <= 5 or maxA >= 4090 or maxB >= 4090
        self.wvals["f"].set(f"{f:,.1f} Hz")
        self.wvals["vppA"].set(f"{vppA:.3f} V")
        self.wvals["vppB"].set(f"{vppB:.3f} V")
        self.wvals["dcA"].set(f"{dcA:.3f} V")
        self.wvals["dcB"].set(f"{dcB:.3f} V")
        self.wvals["rawA"].set(f"{minA} / {maxA}")
        self.wvals["rawB"].set(f"{minB} / {maxB}")
        self.wvals["z"].set(fmt_ohm(z))
        self.wvals["ph"].set(f"{ph:+.2f}°")
        self.wvals["n"].set(f"{n:,}")
        self.wvals["clip"].set("YES - check wiring" if clip else "no")

    # ---------------- Bode plotting ----------------
    def redraw_bode(self, view=None):
        axz, axp = self.ax_z, self.ax_p
        axz.clear(); axp.clear()
        if self.reference:
            rf, rz, rp = self.reference
            axz.plot(rf, rz, "--", color="#ff7f0e", lw=1.2, label="Reference")
            axp.plot(rf, rp, "--", color="#ff7f0e", lw=1.2)
        if self.coarse:
            c = np.array(sorted(self.coarse))
            axz.plot(c[:, 0], c[:, 1], "-", color="#999", lw=1, label="Coarse")
            axp.plot(c[:, 0], c[:, 2], "-", color="#999", lw=1)
        if self.fine:
            d = np.array(sorted(self.fine))
            axz.plot(d[:, 0], d[:, 1], ".-", color="#1f77b4", ms=3, lw=1, label="Zoom")
            axp.plot(d[:, 0], d[:, 2], ".-", color="#1f77b4", ms=3, lw=1)

        r = self.results
        for key, col in (("fs", "#2ca02c"), ("fp", "#d62728")):
            if r.get(key):
                for ax in (axz, axp):
                    ax.axvline(r[key], color=col, lw=1, alpha=0.8)
                axz.text(r[key], 1.0, f" {key}", transform=axz.get_xaxis_transform(),
                         color=col, va="top", fontsize=9)

        axz.set_yscale("log")
        axz.set_ylabel("|Z| (Ω)")
        axz.grid(True, which="both", alpha=0.3)
        axp.set_ylabel("Phase (°)")
        axp.set_xlabel("Frequency (Hz)")
        axp.axhline(0, color="k", lw=0.6)
        axp.grid(True, alpha=0.3)
        axp.set_ylim(-95, 95)
        if axz.get_legend_handles_labels()[0]:
            axz.legend(loc="lower left", fontsize=8)
        title = "Impedance"
        if r.get("fs") and r.get("fp"):
            title += f"   fs = {r['fs']:,.1f} Hz   fp = {r['fp']:,.1f} Hz   Δf = {r['df']:.1f} Hz"
        axz.set_title(title, fontsize=10)

        if view == "zoom" and self.fine:
            fs = [p[0] for p in self.fine]
            axp.set_xlim(min(fs), max(fs))
        elif view == "full" and self.coarse:
            fs = [p[0] for p in self.coarse]
            axp.set_xlim(min(fs), max(fs))
        self.canvas.draw_idle()

    def show_results(self):
        r = self.results
        self.res_vars["fs"].set(fmt_hz(r.get("fs")) if r.get("fs") else "no zero crossing")
        self.res_vars["fp"].set(fmt_hz(r.get("fp")) if r.get("fp") else "-")
        self.res_vars["df"].set(fmt_hz(r.get("df")))
        self.res_vars["zmin"].set(f"{fmt_ohm(r.get('zmin'))} @ {r.get('f_zmin', 0):,.0f}" if "zmin" in r else "-")
        self.res_vars["zmax"].set(f"{fmt_ohm(r.get('zmax'))} @ {r.get('f_zmax', 0):,.0f}" if "zmax" in r else "-")
        self.res_vars["maxphase"].set(f"{r['maxphase']:+.1f}° @ {r['f_maxphase']:,.0f}" if "maxphase" in r else "-")
        self.res_vars["C0"].set(f"{r['C0']*1e9:.3f} nF" if r.get("C0") else "-")
        self.res_vars["keff"].set(f"{r['keff']:.4f}" if r.get("keff") else "-")
        self.res_vars["Qm"].set(f"{r['Qm']:.0f}" if r.get("Qm") else "-")
        self.res_vars["M"].set(f"{r['M']:.1f}" if r.get("M") else "-")

    # ---------------- files ----------------
    def save_csv(self):
        if not (self.coarse or self.fine):
            self.say("Nothing to save yet.")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")],
                                            initialfile=time.strftime("bode_%Y%m%d_%H%M%S.csv"))
        if not path:
            return
        r = self.results
        with open(path, "w", newline="") as fh:
            fh.write(f"# Teensy Bode export {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
            fh.write(f"# RS = {self.rs_var.get()} ohm; {self.cal_label.cget('text')}\n")
            for k in ("fs", "fp", "df", "zmin", "f_zmin", "zmax", "f_zmax", "maxphase", "f_maxphase", "C0", "keff", "Qm", "M"):
                if r.get(k) is not None:
                    fh.write(f"# {k} = {r[k]:.6g}\n")
            w = csv.writer(fh)
            w.writerow(["sweep", "f_Hz", "Zmag_ohm", "phase_deg", "Re_ohm", "Im_ohm"])
            for tag, pts in (("coarse", sorted(self.coarse)), ("zoom", sorted(self.fine))):
                for p in pts:
                    w.writerow([tag] + [f"{v:.6g}" for v in p])
        self.say(f"Saved {path}")

    def load_reference(self):
        path = filedialog.askopenfilename(filetypes=[("CSV", "*.csv"), ("All files", "*.*")])
        if not path:
            return
        f, z, p = [], [], []
        with open(path) as fh:
            for row in csv.reader(line for line in fh if not line.startswith("#")):
                try:
                    if len(row) >= 4 and row[0] in ("coarse", "zoom"):
                        f.append(float(row[1])); z.append(float(row[2])); p.append(float(row[3]))
                    elif len(row) >= 3:
                        f.append(float(row[0])); z.append(float(row[1])); p.append(float(row[2]))
                except ValueError:
                    continue
        if not f:
            messagebox.showerror("Load reference", "No data found in that file.")
            return
        order = np.argsort(f)
        self.reference = (np.array(f)[order], np.array(z)[order], np.array(p)[order])
        self.say(f"Loaded reference: {path} ({len(f)} points)")
        self.redraw_bode()

    def clear_reference(self):
        self.reference = None
        self.redraw_bode()

    # ---------------- event pump ----------------
    def _poll_events(self):
        redraw = False
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "point":
                    tag, vals, i, n = payload
                    (self.coarse if tag == "coarse" else self.fine).append(vals)
                    self.status.config(text=f"{tag} sweep: point {i}/{n}   f = {vals[0]:,.1f} Hz   "
                                            f"|Z| = {fmt_ohm(vals[1])}   phase = {vals[2]:+.1f}°")
                    redraw = True
                elif kind == "status":
                    self.status.config(text=payload)
                elif kind == "log":
                    self.say(payload)
                elif kind == "cal":
                    self.cal_label.config(text=payload)
                elif kind == "wave":
                    self.show_wave(*payload)
                elif kind == "analyze":
                    self.results = analyze(self.fine, self.coarse)
                    self.show_results()
                    self.redraw_bode(view=payload)
                    redraw = False
                elif kind == "done":
                    self.busy = False
                    self.status.config(text="Ready")
                    if self.live_wave.get() and self.link.connected:
                        self.root.after(150, self.capture_wave)
        except queue.Empty:
            pass
        if redraw:
            self.redraw_bode()
        self.root.after(50, self._poll_events)


def main():
    root = tk.Tk()
    try:
        ttk.Style().theme_use("clam")
    except tk.TclError:
        pass
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
