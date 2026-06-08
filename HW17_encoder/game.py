import math
import threading
import pygame

try:
    import serial
    HAVE_SERIAL = True
except ImportError:
    HAVE_SERIAL = False

WIDTH  = 1000
HEIGHT = 700

# ---------------------------------------------------------------------------
# HARDWARE SERIAL CONFIG — set your Pico's port
# ---------------------------------------------------------------------------
SERIAL_PORT = "COM6"        # Windows "COM5"
SERIAL_BAUD = 115200
ENC_TO_THETA_SIGN = -1      # flip to -1 if handle tilts bottle the wrong way
ENC_DEG_OFFSET    = 0.0     # offset (deg) if upright != encoder zero

# data line format from Pico:  D,<force_N>,<angle_deg>\n
hw_force_N   = 0.0
hw_angle_deg = 0.0
hw_connected = False
_hw_lock = threading.Lock()


def _serial_worker():
    global hw_force_N, hw_angle_deg, hw_connected
    if not HAVE_SERIAL:
        print("[serial] pyserial not installed; SIM mode. (pip install pyserial)")
        return
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
    except Exception as e:
        print(f"[serial] could not open {SERIAL_PORT}: {e}")
        print("[serial] SIM mode (keyboard A/D for tilt).")
        return
    print(f"[serial] connected {SERIAL_PORT} @ {SERIAL_BAUD}")
    with _hw_lock:
        hw_connected = True
    buf = b""
    while True:
        try:
            chunk = ser.read(128)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", errors="ignore").strip()
                if not text.startswith("D,"):
                    continue
                parts = text.split(",")
                if len(parts) != 3:
                    continue
                try:
                    f = float(parts[1])
                    a = float(parts[2])
                except ValueError:
                    continue
                with _hw_lock:
                    hw_force_N = f
                    hw_angle_deg = a
        except Exception:
            continue


threading.Thread(target=_serial_worker, daemon=True).start()


# ---------------------------------------------------------------------------
# Bottle geometry (wine-bottle profile)
# ---------------------------------------------------------------------------
R_BODY     = 0.0375
R_NECK     = 0.007
H_BODY     = 0.155
H_SHOULDER = 0.045
H_NECK     = 0.075
H_TOTAL    = H_BODY + H_SHOULDER + H_NECK

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
theta      = 0.0            # bottle tilt (rad); +theta leans right
TILT_SPEED = math.radians(75)   # keyboard fallback speed

PPM        = 1300.0
PIVOT_Y    = HEIGHT * 0.58

PLOT_LEN   = 320
force_hist = [0.0] * PLOT_LEN
LC_FORCE_MAX = 30.0         # plot scale reference


def render_angle(th):
    return -th


def radius_at(y):
    if y <= 0.0:
        return R_BODY
    if y <= H_BODY:
        return R_BODY
    if y <= H_BODY + H_SHOULDER:
        t = (y - H_BODY) / H_SHOULDER
        return R_BODY + t * (R_NECK - R_BODY)
    if y <= H_TOTAL:
        return R_NECK
    return 0.0


def read_load_cell():
    with _hw_lock:
        return hw_force_N if hw_connected else 0.0


# ---------------------------------------------------------------------------
# Update
# ---------------------------------------------------------------------------
def update(dt):
    global theta

    with _hw_lock:
        connected = hw_connected
        enc_deg = hw_angle_deg
        force = hw_force_N

    if connected:
        theta = math.radians(ENC_TO_THETA_SIGN * enc_deg + ENC_DEG_OFFSET)
    else:
        # keyboard fallback so you can test layout without hardware
        if keyboard.d:
            theta += TILT_SPEED * dt
        if keyboard.a:
            theta -= TILT_SPEED * dt

    theta = max(math.radians(-180), min(math.radians(180), theta))

    force_hist.append(force if connected else 0.0)
    force_hist.pop(0)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def local_to_px(p_local, pivot_px, thr):
    x, y = p_local
    c, s = math.cos(thr), math.sin(thr)
    wx = x * c - y * s
    wy = x * s + y * c
    return (pivot_px[0] + wx * PPM, pivot_px[1] - wy * PPM)


def bottle_outline_points():
    ys = []
    n_body, n_sh, n_neck = 6, 10, 4
    for i in range(n_body + 1):
        ys.append(i / n_body * H_BODY)
    for i in range(1, n_sh + 1):
        ys.append(H_BODY + i / n_sh * H_SHOULDER)
    for i in range(1, n_neck + 1):
        ys.append(H_BODY + H_SHOULDER + i / n_neck * H_NECK)
    right = [(radius_at(y), y) for y in ys]
    left = [(-radius_at(y), y) for y in reversed(ys)]
    return right + left


def draw_plot(x, y, w, h, data, ymax, color, label):
    screen.draw.filled_rect(Rect((x, y), (w, h)), (24, 26, 34))
    screen.draw.rect(Rect((x, y), (w, h)), (70, 75, 90))
    zero_y = y + h / 2
    screen.draw.line((x, zero_y), (x + w, zero_y), (55, 60, 72))
    screen.draw.text(label, (x + 6, y + 4), color=color, fontsize=18)
    n = len(data)
    if n < 2 or ymax <= 0:
        return
    pts = []
    for i, v in enumerate(data):
        px = x + (i / (n - 1)) * w
        norm = max(-1.0, min(1.0, v / ymax))
        py = zero_y - norm * (h / 2 - 6)
        pts.append((px, py))
    for i in range(len(pts) - 1):
        screen.draw.line(pts[i], pts[i + 1], color)


def draw():
    screen.fill((16, 18, 26))

    with _hw_lock:
        connected = hw_connected
        force = hw_force_N
        enc_deg = hw_angle_deg

    thr = render_angle(theta)
    pivot_px = (WIDTH * 0.32, PIVOT_Y)

    # ground line under the bottle
    screen.draw.line((0, PIVOT_Y + 6), (WIDTH * 0.62, PIVOT_Y + 6), (70, 60, 50))

    # bottle silhouette
    outline = bottle_outline_points()
    outline_px = [local_to_px(p, pivot_px, thr) for p in outline]
    pygame.draw.polygon(screen.surface, (40, 55, 48), outline_px)
    pygame.draw.polygon(screen.surface, (120, 140, 130), outline_px, 2)
    screen.draw.filled_circle(pivot_px, 4, (210, 210, 210))

    # readouts
    src = "ENCODER" if connected else "keyboard (A/D)"
    screen.draw.text(f"tilt  : {math.degrees(theta):6.1f} deg   [{src}]",
                     (40, 30), color=(220, 225, 235), fontsize=24)
    screen.draw.text(f"force : {force:7.2f} N",
                     (40, 62), color=(240, 220, 90), fontsize=24)

    badge = "HW CONNECTED" if connected else "SIM (no hardware)"
    bcol = (120, 220, 140) if connected else (200, 160, 90)
    screen.draw.text(badge, (WIDTH - 230, 30), color=bcol, fontsize=20)

    # live force plot at the bottom
    plot_w = WIDTH - 80
    plot_h = 150
    plot_x = 40
    plot_y = HEIGHT - plot_h - 30
    fo_max = max(1.0, max(abs(v) for v in force_hist) * 1.1)
    draw_plot(plot_x, plot_y, plot_w, plot_h, force_hist, fo_max,
              (240, 220, 90), f"load cell force (N)   range +/-{fo_max:.1f}")