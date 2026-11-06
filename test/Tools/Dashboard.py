"""!
@file Dashboard.py
@brief Live dashboard for the ESP32 Enthalyzer

A Python script to log, parse, and visualize the 22-column timestamped 
raw serial data from the ESP32 Enthalyzer in real-time. 

v6.0 - 22-column output. Gas-side U. 
       Energy panel now compares Q_latent AND Q_water_gain.
       Flow panel shows RAW flow + a 10-sample rolling average.
       CSV logs separate 'date' and 'time' columns.

@author Ayur Jitendra Ghalot
@contact ayur.ghalot@tu-dresden.de
@copyright Copyright (c) 2026 Ayur Jitendra Ghalot, TU Dresden, Chair of Bioprocess Engineering.

@license MIT License

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

@date 2026-11-06
@version 1.0.0
"""

import sys
import time
import csv
import threading
from collections import deque
from datetime import datetime

import numpy as np
import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button

# ============================================================
# CONFIGURATION
# ============================================================
PORT               = "COM3" #Select the correct COM port for your ESP32
BAUD               = 115200
CONNECT_TIMEOUT_S  = 30
WINDOW_MIN         = 15
MAXPTS             = 8000
STALE_TIMEOUT_S    = 20
FLOW_AVG_N         = 10          
LOG_FILE           = f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
SNAPSHOT_FMT       = "snapshot_{ts}.png"

# 22 columns - MUST match Arduino header exactly, in order
COLS = [
    "Runtime_ms",
    "Flow_Lpm_raw",       
    "Flow_Lpm",           
    "T_water_in_C",
    "T_water_out_C",
    "In_rH", "In_T_C", "h_in_kJkg",
    "Out_rH", "Out_T_C", "h_out_kJkg",
    "LMED_kJkg",
    "w_in_gkg", "w_out_gkg", "dW_gkg", "DehumEff_pct",
    "Q_sensible_W", "Q_latent_W", "Q_total_W",
    "Condensate_gph",
    "U_gas",
    "Q_water_gain_W",     # NEW COLUMN ADDED
]

# ============================================================
# STYLING
# ============================================================
plt.style.use("dark_background")
plt.rcParams.update({
    "figure.facecolor": "#0e1117",
    "axes.facecolor":   "#161b22",
    "axes.edgecolor":   "#30363d",
    "axes.labelcolor":  "#c9d1d9",
    "xtick.color":      "#8b949e",
    "ytick.color":      "#8b949e",
    "grid.color":       "#30363d",
    "grid.alpha":       0.35,
    "text.color":       "#c9d1d9",
    "font.family":      "DejaVu Sans",
    "font.size":        9,
    "axes.titlesize":   10,
    "axes.titleweight": "bold",
    "legend.facecolor": "#161b22",
    "legend.edgecolor": "#30363d",
    "legend.framealpha":0.85,
})

PALETTE = {
    "w_in":    "#58a6ff", "w_out":   "#f0883e",
    "air_in":  "#f85149", "air_out": "#ff9e64",
    "delta":   "#bc8cff",
    "rh_in":   "#79c0ff", "rh_out":  "#56d364",
    "h_in":    "#d2a8ff", "h_out":   "#ffa657", "lmed": "#8b949e",
    "flow":    "#39d0d8", "cond":    "#7ee787",
    "dw":      "#39d0d8", "dehum":   "#56d364",
    "q_lat":   "#58a6ff", "q_water": "#a2d2ff",
    "u_coeff": "#e3b341",
}

# ============================================================
# 1) CONNECTION CHECK
# ============================================================
def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

def open_and_verify(port, baud):
    print(f"[INFO] Available ports: {list_ports()}")
    print(f"[INFO] Opening {port} @ {baud} ...")
    try:
        ser = serial.Serial(port, baud, timeout=1)
    except serial.SerialException as e:
        print(f"[FATAL] Could not open {port}: {e}")
        sys.exit(1)

    time.sleep(2.0)
    ser.reset_input_buffer()
    print("[INFO] Waiting for first CSV frame ...")
    start = time.time()
    first_frame = None

    while time.time() - start < CONNECT_TIMEOUT_S:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        print(f"  > {line}")
        parts = [p.strip() for p in line.split(";")]
        if len(parts) == len(COLS):
            try:
                [float(x) for x in parts]
                first_frame = parts
                break
            except ValueError:
                pass

    if first_frame is None:
        print(f"[FATAL] No valid data frame received in {CONNECT_TIMEOUT_S}s.")
        print(f"        Expected {len(COLS)} ';'-separated fields per row.")
        ser.close()
        sys.exit(1)

    print("[ OK ] Connection verified. First frame:")
    for name, val in zip(COLS, first_frame):
        print(f"        {name:<16} = {val}")
    return ser, first_frame

# ============================================================
# 2) STATE
# ============================================================
buffers     = {c: deque(maxlen=MAXPTS) for c in COLS}
t_wall      = deque(maxlen=MAXPTS)
t0          = time.time()
_lock       = threading.Lock()
paused      = [False]
last_rx_ts  = [time.time()]
frame_count = [0]

# ============================================================
# 3) READER THREAD
# ============================================================
def reader_thread(ser, first_frame, csv_writer):
    def push(parts):
        with _lock:
            t_wall.append(time.time() - t0)
            for name, val in zip(COLS, parts):
                buffers[name].append(float(val))
            last_rx_ts[0] = time.time()
            frame_count[0] += 1
        now = datetime.now()
        csv_writer.writerow(
            [now.strftime("%Y-%m-%d"), now.strftime("%H:%M:%S")] + parts
        )

    push(first_frame)
    while True:
        try:
            line = ser.readline().decode(errors="ignore").strip()
        except Exception:
            continue
        if not line:
            continue
        parts = [p.strip() for p in line.split(";")]
        if len(parts) != len(COLS):
            print(f"[esp32] {line}")
            continue
        try:
            if not paused[0]:
                push(parts)
        except ValueError:
            print(f"[warn] non-numeric row: {line}")

# ============================================================
# 4) FOOTER / MATH HELPERS
# ============================================================
def live_val(values, unit="", fmt="{:.2f}"):
    if not values: return "--"
    return f"{fmt.format(values[-1])}{unit}"

def rolling_mean(values, w):
    a = np.asarray(values, dtype=float)
    if a.size == 0: return a
    c   = np.cumsum(np.insert(a, 0, 0.0))
    idx = np.arange(a.size)
    lo  = np.maximum(0, idx - w + 1)
    return (c[idx + 1] - c[lo]) / (idx - lo + 1)

def flow_val(values, unit=" L/min", fmt="{:.2f}", n=FLOW_AVG_N):
    if not values: return "--"
    arr = np.asarray(values)
    avg = arr[-n:].mean()
    return (f"raw={fmt.format(arr[-1])}{unit}  "
            f"avg{n}={fmt.format(avg)}{unit}")

# ============================================================
# 5) BUILD FIGURE 
# ============================================================
def build_figure():
    fig = plt.figure(figsize=(16, 11))
    fig.canvas.manager.set_window_title("ESP32 Enthalyzer - Dehumidification Dashboard")

    gs = fig.add_gridspec(
        nrows=4, ncols=2,
        left=0.06, right=0.94, top=0.90, bottom=0.09,
        hspace=0.55, wspace=0.28,
    )

    ax_T  = fig.add_subplot(gs[0, 0])   # Water + Air Temperatures
    ax_dT = fig.add_subplot(gs[0, 1])   # Air-side dT
    ax_RH = fig.add_subplot(gs[1, 0])   # Relative Humidity
    ax_H  = fig.add_subplot(gs[1, 1])   # Enthalpy + LMED
    ax_F  = fig.add_subplot(gs[2, 0])   # Water Flow (raw+avg10) + Condensate
    ax_D  = fig.add_subplot(gs[2, 1])   # dW + DehumEff
    ax_B  = fig.add_subplot(gs[3, 0])   # Energy (Q_latent + Q_water_gain)
    ax_U  = fig.add_subplot(gs[3, 1])   # Gas-side U

    all_axes = [ax_T, ax_dT, ax_RH, ax_H, ax_F, ax_D, ax_B, ax_U]

    # ---- Panel 1: Water In/Out + Air Temperatures ----
    ln_win, = ax_T.plot([], [], color=PALETTE["w_in"],   lw=2.0, label="H2O In")
    ln_wout,= ax_T.plot([], [], color=PALETTE["w_out"],  lw=2.0, label="H2O Out")
    ln_tin, = ax_T.plot([], [], color=PALETTE["air_in"], lw=1.4, ls="--", label="Air In")
    ln_tou, = ax_T.plot([], [], color=PALETTE["air_out"],lw=1.4, ls="--", label="Air Out")
    ax_T.set_title("Water & Air Temperatures")
    ax_T.set_ylabel("degC")
    ax_T.legend(loc="upper left", fontsize=7, ncol=4)

    # ---- Panel 2: Air-side dT ----
    ln_dT,  = ax_dT.plot([], [], color=PALETTE["delta"], lw=2.0, label="dT air (in-out)")
    ax_dT.set_title("Air-Side dT")
    ax_dT.set_ylabel("degC")
    ax_dT.axhline(0, color="#30363d", lw=0.8)
    ax_dT.legend(loc="upper left", fontsize=8)

    # ---- Panel 3: Relative Humidity ----
    ln_rhi, = ax_RH.plot([], [], color=PALETTE["rh_in"],  lw=1.8, label="Inlet RH")
    ln_rho, = ax_RH.plot([], [], color=PALETTE["rh_out"], lw=1.8, label="Outlet RH")
    ax_RH.set_title("Relative Humidity")
    ax_RH.set_ylabel("%")
    ax_RH.set_ylim(0, 105)
    ax_RH.legend(loc="upper left", fontsize=8, ncol=2)

    # ---- Panel 4: Enthalpies + LMED ----
    ln_hi,  = ax_H.plot([], [], color=PALETTE["h_in"],  lw=1.8, label="h inlet")
    ln_ho,  = ax_H.plot([], [], color=PALETTE["h_out"], lw=1.8, label="h outlet")
    ln_lm,  = ax_H.plot([], [], color=PALETTE["lmed"],  lw=1.6, ls="--", label="LMED")
    ax_H.set_title("Enthalpy & LMED (signed)")
    ax_H.set_ylabel("kJ/kg")
    ax_H.axhline(0, color="#30363d", lw=0.8)
    ax_H.legend(loc="upper left", fontsize=8, ncol=3)

    # ---- Panel 5: Water Flow RAW + avg (left) + Condensate (right twin) ----
    ln_flow_raw, = ax_F.plot([], [], color=PALETTE["flow"], lw=1.0, alpha=0.45, label="Flow raw (L/min)")
    ln_flow_avg, = ax_F.plot([], [], color="#ffffff", lw=2.0, label=f"Flow avg-{FLOW_AVG_N} (L/min)")
    ax_F.set_title("Water Flow & Condensate Rate")
    ax_F.set_ylabel("L/min", color=PALETTE["flow"])
    ax_F.tick_params(axis="y", labelcolor=PALETTE["flow"])
    ax_F.spines["left"].set_color(PALETTE["flow"])
    ax_F2 = ax_F.twinx()
    ln_cond,= ax_F2.plot([], [], color=PALETTE["cond"], lw=2.0, label="Condensate (g/h)")
    ax_F2.set_ylabel("g/h", color=PALETTE["cond"])
    ax_F2.tick_params(axis="y", labelcolor=PALETTE["cond"])
    ax_F2.spines["right"].set_color(PALETTE["cond"])
    ax_F.legend(handles=[ln_flow_raw, ln_flow_avg, ln_cond],
                labels=[ln_flow_raw.get_label(), ln_flow_avg.get_label(), ln_cond.get_label()],
                loc="upper left", fontsize=8, ncol=3)

    # ---- Panel 6: Moisture removed (left) + DehumEff (right twin) ----
    ln_dw,  = ax_D.plot([], [], color=PALETTE["dw"], lw=2.4, label="dW (g/kg)")
    ax_D.set_title("DEHUMIDIFICATION: Moisture Removed & Efficiency")
    ax_D.set_ylabel("dW  [g/kg]", color=PALETTE["dw"])
    ax_D.tick_params(axis="y", labelcolor=PALETTE["dw"])
    ax_D.spines["left"].set_color(PALETTE["dw"])
    ax_D.axhline(0, color="#30363d", lw=0.6)
    ax_D2 = ax_D.twinx()
    ln_de,  = ax_D2.plot([], [], color=PALETTE["dehum"], lw=2.4, label="Dehum. Eff (%)")
    ax_D2.set_ylabel("Dehum. Eff [%]", color=PALETTE["dehum"])
    ax_D2.tick_params(axis="y", labelcolor=PALETTE["dehum"])
    ax_D2.spines["right"].set_color(PALETTE["dehum"])
    ax_D2.set_ylim(0, 110)
    ax_D2.axhline(100, color="#30363d", ls=":", lw=0.8)
    ax_D.legend(handles=[ln_dw, ln_de], labels=[ln_dw.get_label(), ln_de.get_label()],
                loc="upper left", fontsize=8, ncol=2)

    # ---- Panel 7: Energy (Q_latent vs Q_water_gain) ----
    ln_ql,  = ax_B.plot([], [], color=PALETTE["q_lat"], lw=2.2, label="Q latent")
    ln_qw,  = ax_B.plot([], [], color=PALETTE["q_water"], lw=2.2, ls="--", label="Q water gain")
    ax_B.set_title("Energy: Q Latent vs Q Water Gain")
    ax_B.set_ylabel("W")
    ax_B.axhline(0, color="#30363d", lw=0.6)
    ax_B.legend(loc="upper left", fontsize=8, ncol=2)

    # ---- Panel 8: Gas-side Overall U ----
    ln_U,   = ax_U.plot([], [], color=PALETTE["u_coeff"], lw=2.2, label="U gas [W/(m2.kJ/kg)]")
    ax_U.set_title("Gas-Side Overall U  (Q_total / A.F.LMED)")
    ax_U.set_ylabel("U  [W/(m2.kJ/kg)]", color=PALETTE["u_coeff"])
    ax_U.tick_params(axis="y", labelcolor=PALETTE["u_coeff"])
    ax_U.spines["left"].set_color(PALETTE["u_coeff"])
    ax_U.axhline(0, color="#30363d", lw=0.6)
    ax_U.legend(loc="upper left", fontsize=8)

    for ax in all_axes:
        ax.grid(True, alpha=0.3)
        ax.set_xlabel("Time (min)", fontsize=8)

    # ============================================================
    # HEADER & FOOTER
    # ============================================================
    fig.suptitle("ESP32 Enthalyzer - Live Dehumidification Dashboard",
                 fontsize=13, fontweight="bold", color="#f0f6fc", x=0.06, y=0.975, ha="left")

    info_text = fig.text(0.30, 0.958, "", fontsize=9, color="#8b949e", ha="left")
    status_text = fig.text(0.94, 0.958, "CONNECTING...", fontsize=10, fontweight="bold", 
                           color="#f0b429", ha="right", va="center",
                           bbox=dict(boxstyle="round,pad=0.4", facecolor="#161b22", edgecolor="#30363d", alpha=0.9))

    stats_box = fig.text(0.50, 0.020, "", fontsize=8, color="#c9d1d9", ha="center", family="DejaVu Sans Mono",
                         bbox=dict(boxstyle="round,pad=0.5", facecolor="#161b22", edgecolor="#30363d", alpha=0.9))

    lines = {
        "T_water_in_C":  ln_win,  "T_water_out_C": ln_wout,
        "In_T_C": ln_tin, "Out_T_C": ln_tou, "dT": ln_dT,
        "In_rH": ln_rhi, "Out_rH": ln_rho,
        "h_in_kJkg": ln_hi, "h_out_kJkg": ln_ho, "LMED_kJkg": ln_lm,
        "Flow_Lpm_raw": ln_flow_raw, "Flow_avg10": ln_flow_avg,
        "Condensate_gph": ln_cond,
        "dW_gkg": ln_dw, "DehumEff_pct": ln_de,
        "Q_latent_W": ln_ql, "Q_water_gain_W": ln_qw,
        "U_gas": ln_U,
    }
    twins = {"ax_F2": ax_F2, "ax_D2": ax_D2}

    return (fig, lines, all_axes, twins, status_text, info_text, stats_box)

# ============================================================
# 6) UPDATER
# ============================================================
def make_updater(lines, all_axes, twins, status_text, info_text, stats_box):

    def autoscale_y(ax, values_list, pad_frac=0.10, floor=None, ceil=None):
        flat = []
        for v in values_list:
            if v: flat.extend(v)
        if not flat: return
        lo, hi = min(flat), max(flat)
        if lo == hi: lo -= 0.5; hi += 0.5
        span = hi - lo
        lo -= span * pad_frac
        hi += span * pad_frac
        if floor is not None: lo = max(lo, floor)
        if ceil  is not None: hi = min(hi, ceil)
        ax.set_ylim(lo, hi)

    ax_F2 = twins["ax_F2"]; ax_D2 = twins["ax_D2"]

    def update(_):
        with _lock:
            if not t_wall: return
            xs = np.array(t_wall) / 60.0
            snap = {k: list(v) for k, v in buffers.items()}
            n_frames = frame_count[0]

        age = time.time() - last_rx_ts[0]
        if paused[0]:
            status_text.set_text("PAUSED");  status_text.set_color("#f0b429")
        elif age > STALE_TIMEOUT_S:
            status_text.set_text(f"STALE ({age:.0f}s)"); status_text.set_color("#f85149")
        else:
            status_text.set_text(f"LIVE ({age:.1f}s)");  status_text.set_color("#56d364")

        info_text.set_text(f"frames: {n_frames}   |   port: {PORT}   |   log: {LOG_FILE}   |   {datetime.now():%H:%M:%S}")

        for key, ln in lines.items():
            if key == "dT":
                if snap["In_T_C"] and snap["Out_T_C"]:
                    dT = np.array(snap["In_T_C"]) - np.array(snap["Out_T_C"])
                    ln.set_data(xs, dT)
            elif key == "Flow_avg10":
                if snap["Flow_Lpm_raw"]:
                    ln.set_data(xs, rolling_mean(snap["Flow_Lpm_raw"], FLOW_AVG_N))
            else:
                ln.set_data(xs, snap[key])

        xmax = xs[-1]; xmin = max(0, xmax - WINDOW_MIN)
        for ax in all_axes: ax.set_xlim(xmin, xmax + 0.05)
        for tw in (ax_F2, ax_D2): tw.set_xlim(xmin, xmax + 0.05)

        ax_T, ax_dT, ax_RH, ax_H, ax_F, ax_D, ax_B, ax_U = all_axes
        autoscale_y(ax_T,  [snap["T_water_in_C"], snap["T_water_out_C"], snap["In_T_C"], snap["Out_T_C"]])
        if snap["In_T_C"] and snap["Out_T_C"]:
            dT_arr = np.array(snap["In_T_C"]) - np.array(snap["Out_T_C"])
            autoscale_y(ax_dT, [list(dT_arr)])
        autoscale_y(ax_H,  [snap["h_in_kJkg"], snap["h_out_kJkg"], snap["LMED_kJkg"]])
        autoscale_y(ax_F,  [snap["Flow_Lpm_raw"]], floor=0)   
        autoscale_y(ax_F2, [snap["Condensate_gph"]], floor=0)
        autoscale_y(ax_D,  [snap["dW_gkg"]])
        autoscale_y(ax_B,  [snap["Q_latent_W"], snap["Q_water_gain_W"]])
        autoscale_y(ax_U,  [snap["U_gas"]])

        # ---- Footer: Added Q_water_gain read-out ----
        s = (
            f"H2O_in: {live_val(snap['T_water_in_C'],' C','{:6.2f}')}  |  "
            f"H2O_out: {live_val(snap['T_water_out_C'],' C','{:6.2f}')}  |  "
            f"Flow: {flow_val(snap['Flow_Lpm_raw'],' L/min','{:5.2f}')}\n"
            f"dW: {live_val(snap['dW_gkg'],' g/kg','{:6.2f}')}  |  "
            f"DehumEff: {live_val(snap['DehumEff_pct'],' %','{:5.1f}')}  |  "
            f"Condensate: {live_val(snap['Condensate_gph'],' g/h','{:6.2f}')}\n"
            f"Q_latent: {live_val(snap['Q_latent_W'],' W','{:6.3f}')}  |  "
            f"U_gas: {live_val(snap['U_gas'],'','{:7.1f}')}  |  "
            f"Q_water: {live_val(snap['Q_water_gain_W'],' W','{:6.3f}')}"
        )
        stats_box.set_text(s)

    return update

# ============================================================
# 7) CONTROL BUTTONS
# ============================================================
def add_controls(fig):
    ax_pause  = fig.add_axes([0.06, 0.005, 0.09, 0.030])
    ax_snap   = fig.add_axes([0.16, 0.005, 0.09, 0.030])
    ax_clear  = fig.add_axes([0.26, 0.005, 0.09, 0.030])

    btn_pause = Button(ax_pause, "Pause",    color="#21262d", hovercolor="#30363d")
    btn_snap  = Button(ax_snap,  "Snapshot", color="#21262d", hovercolor="#30363d")
    btn_clear = Button(ax_clear, "Clear",    color="#21262d", hovercolor="#30363d")

    for b in (btn_pause, btn_snap, btn_clear):
        b.label.set_color("#c9d1d9")
        b.label.set_fontsize(9)

    def on_pause(_):
        paused[0] = not paused[0]
        btn_pause.label.set_text("Resume" if paused[0] else "Pause")
        fig.canvas.draw_idle()

    def on_snap(_):
        fn = SNAPSHOT_FMT.format(ts=datetime.now().strftime("%Y%m%d_%H%M%S"))
        fig.savefig(fn, dpi=150, facecolor=fig.get_facecolor())
        print(f"[INFO] Snapshot saved -> {fn}")

    def on_clear(_):
        with _lock:
            t_wall.clear()
            for k in buffers: buffers[k].clear()
        print("[INFO] Buffers cleared")

    btn_pause.on_clicked(on_pause)
    btn_snap.on_clicked(on_snap)
    btn_clear.on_clicked(on_clear)
    fig._widgets = [btn_pause, btn_snap, btn_clear]

# ============================================================
# 8) MAIN
# ============================================================
def main():
    ser, first_frame = open_and_verify(PORT, BAUD)

    log = open(LOG_FILE, "w", newline="", encoding="utf-8")
    writer = csv.writer(log)
    writer.writerow(["date", "time"] + COLS)     
    print(f"[INFO] Logging raw frames to {LOG_FILE}")

    t = threading.Thread(target=reader_thread, args=(ser, first_frame, writer), daemon=True)
    t.start()

    (fig, lines, all_axes, twins, status_text, info_text, stats_box) = build_figure()
    add_controls(fig)

    ani = FuncAnimation(
        fig,
        make_updater(lines, all_axes, twins, status_text, info_text, stats_box),
        interval=1000, blit=False, cache_frame_data=False,
    )

    try:
        plt.show()
    finally:
        final_png = SNAPSHOT_FMT.format(ts=datetime.now().strftime("%Y%m%d_%H%M%S_final"))
        try:
            fig.savefig(final_png, dpi=150, facecolor=fig.get_facecolor())
        except Exception:
            pass
        log.flush(); log.close()
        ser.close()

if __name__ == "__main__":
    main()
