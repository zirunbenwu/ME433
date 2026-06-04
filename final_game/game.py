import math
import pygame

WIDTH  = 1100
HEIGHT = 900          

# ---------------------------------------------------------------------------
# Physical parameters
# ---------------------------------------------------------------------------
RHO       = 1000.0
G         = 9.81

R_BODY     = 0.0375
R_NECK     = 0.007        # smaller opening -> glug starts sooner
H_BODY     = 0.155
H_SHOULDER = 0.045
H_NECK     = 0.075
H_TOTAL    = H_BODY + H_SHOULDER + H_NECK
LIP_Y      = H_TOTAL

M_BOTTLE   = 0.45
XB_LOCAL   = 0.0
YB_LOCAL   = H_TOTAL * 0.45

K_POUR     = 4.0e-4

THETA_WALL = math.radians(-10.0)
K_WALL     = 6.0
B_WALL     = 0.05

# --- slosh ---
L_EFF      = 0.05
ZETA       = 0.08
ALPHA      = 0.015
K_S_TORQUE = 8.0
VIS_TILT   = 6.0
slosh_gain = 1.0

# --- glug ---
# Discrete decaying pulses fired at a low rate while the opening is submerged.
# Each pulse: A * sin(2*pi*F_GLUG*t) * exp(-t/TAU_GLUG)  -> a heavy "thunk".
F_GLUG       = 14.0       # in-pulse oscillation freq (Hz) - low = heavy
TAU_GLUG     = 0.055      # pulse decay time (s) - short ring
A_GLUG       = 0.45       # pulse amplitude (N*m) - high
GLUG_RATE_K  = 9.0        # glugs per second per unit flow (sets spacing)
GLUG_RATE_MIN = 1.5       # floor rate so it still glugs when nearly empty (Hz)
GLUG_RATE_MAX = 5.0       # cap rate (Hz) - low frequency of events

# ---------------------------------------------------------------------------
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


def clamp01(v):
    return 0.0 if v < 0 else (1.0 if v > 1 else v)


def _full_volume():
    N = 600
    dy = H_TOTAL / N
    v = 0.0
    for i in range(N):
        y = (i + 0.5) * dy
        r = radius_at(y)
        v += math.pi * r * r * dy
    return v

V_MAX = _full_volume()

# ---------------------------------------------------------------------------
# State
# ---------------------------------------------------------------------------
theta      = 0.0
theta_dot  = 0.0
theta_dot_prev = 0.0
theta_ddot_f = 0.0
bottle_x   = 0.30
V          = V_MAX
paused     = False

s_slosh    = 0.0
sd_slosh   = 0.0

# glug state
glug_timer   = 0.0        # counts down to next glug event
glug_phase   = 1e9        # time since last pulse started (big = inactive)
glug_active  = False
glug_gain    = 1.0        # LIVE multiplier, tuned with ; and '

TILT_SPEED = math.radians(75)
MOVE_SPEED = 0.45

PPM         = 1300.0
PIVOT_Y     = HEIGHT * 0.42
GLASS_X     = 0.66
GLASS_TOP_Y = HEIGHT * 0.56


# ---------------------------------------------------------------------------
# Geometry / liquid
# ---------------------------------------------------------------------------
def world_h(p_local, thr):
    x, y = p_local
    return x * math.sin(thr) + y * math.cos(thr)


def _vol_below(surf, thr, N=240):
    s, c = math.sin(thr), math.cos(thr)
    dy = H_TOTAL / N
    v = 0.0
    for i in range(N):
        y = (i + 0.5) * dy
        r = radius_at(y)
        if r <= 0:
            continue
        if abs(s) < 1e-9:
            frac = 1.0 if (y * c <= surf) else 0.0
        else:
            x_cross = (surf - y * c) / s
            frac = (clamp01((x_cross + r) / (2 * r)) if s > 0
                    else clamp01((r - x_cross) / (2 * r)))
        v += frac * math.pi * r * r * dy
    return v


def liquid_surface(V_cur, thr):
    lo, hi = -R_BODY - H_TOTAL, R_BODY + H_TOTAL
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if _vol_below(mid, thr) < V_cur:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def lip_lowest_world(thr):
    s, c = math.sin(thr), math.cos(thr)
    return LIP_Y * c - R_NECK * abs(s)


def lip_highest_world(thr):
    s, c = math.sin(thr), math.cos(thr)
    return LIP_Y * c + R_NECK * abs(s)


def liquid_com(V_cur, thr, N=240):
    if V_cur <= 0.0:
        return 0.0, 0.0, 0.0
    surf = liquid_surface(V_cur, thr)
    s, c = math.sin(thr), math.cos(thr)
    dy = H_TOTAL / N
    sx = sy = sw = 0.0
    for i in range(N):
        y = (i + 0.5) * dy
        r = radius_at(y)
        if r <= 0:
            continue
        if abs(s) < 1e-9:
            frac = 1.0 if (y * c <= surf) else 0.0
            xc = 0.0
        else:
            x_cross = (surf - y * c) / s
            if s > 0:
                frac = clamp01((x_cross + r) / (2 * r))
                xc = 0.5 * (-r + min(x_cross, r))
            else:
                frac = clamp01((r - x_cross) / (2 * r))
                xc = 0.5 * (max(x_cross, -r) + r)
        w = frac * math.pi * r * r * dy
        sx += xc * w
        sy += y * w
        sw += w
    if sw <= 0:
        return 0.0, 0.0, surf
    return sx / sw, sy / sw, surf


def wall_torque(th, th_dot):
    if th < THETA_WALL:
        penetration = THETA_WALL - th
        tau = K_WALL * penetration - B_WALL * th_dot
        return max(0.0, tau)
    return 0.0


def integrate_slosh(dt, thr):
    global s_slosh, sd_slosh
    if dt <= 0:
        return
    k_s = max(0.5, G * abs(math.cos(thr)) / L_EFF)
    c_s = 2.0 * ZETA * math.sqrt(k_s)
    drive = ALPHA * theta_ddot_f
    sdd = -k_s * s_slosh - c_s * sd_slosh + drive
    sd_slosh += sdd * dt
    s_slosh  += sd_slosh * dt


# ---------------------------------------------------------------------------
# Glug: detect submerged opening + flow, schedule discrete pulses,
# return current pulse torque.
# ---------------------------------------------------------------------------
def update_glug(dt, thr, flow_rate):
    global glug_timer, glug_phase, glug_active

    # opening submerged? whole rim below the liquid surface
    surf = liquid_surface(V, thr) if V > 0 else -1e9
    lip_hi = lip_highest_world(thr)
    submerged = (V > 0) and (surf > lip_hi) and (flow_rate > 1e-9)
    glug_active = submerged

    if dt <= 0:
        return 0.0

    if submerged:
        # event rate scales with flow, clamped to a low band
        rate = GLUG_RATE_K * flow_rate * 1e6     # flow is tiny m^3/s -> scale
        rate = max(GLUG_RATE_MIN, min(GLUG_RATE_MAX, rate))
        glug_timer -= dt
        if glug_timer <= 0.0:
            glug_phase = 0.0                     # start a new pulse
            glug_timer = 1.0 / rate              # schedule next
    else:
        glug_timer = 0.0                          # ready to glug immediately next time

    # advance current pulse
    glug_phase += dt
    if glug_phase < 6 * TAU_GLUG:                 # pulse still ringing
        tau = (A_GLUG * glug_gain
               * math.sin(2 * math.pi * F_GLUG * glug_phase)
               * math.exp(-glug_phase / TAU_GLUG))
        return tau
    return 0.0


# ---------------------------------------------------------------------------
# Derived physics
# ---------------------------------------------------------------------------
def compute_state():
    thr = render_angle(theta)

    m_liquid = RHO * V
    m_total  = M_BOTTLE + m_liquid

    xl, yl, surf = liquid_com(V, thr)
    x_com = (M_BOTTLE * XB_LOCAL + m_liquid * xl) / m_total
    y_com = (M_BOTTLE * YB_LOCAL + m_liquid * yl) / m_total

    world_x = x_com * math.cos(thr) - y_com * math.sin(thr)
    tau_grav = -m_total * G * world_x

    tau_w = wall_torque(theta, theta_dot)

    mass_frac = m_liquid / (RHO * V_MAX) if V_MAX > 0 else 0.0
    tau_slosh = K_S_TORQUE * slosh_gain * s_slosh * mass_frac

    return (m_liquid, m_total, x_com, y_com,
            tau_grav, tau_w, tau_slosh, surf, thr)


# ---------------------------------------------------------------------------
# Update
# ---------------------------------------------------------------------------
def update(dt):
    global theta, theta_dot, theta_dot_prev, theta_ddot_f, bottle_x, V
    global last_tau_glug

    prev_theta = theta

    if keyboard.d:
        theta += TILT_SPEED * dt
    if keyboard.a:
        theta -= TILT_SPEED * dt

    MAX_PENETRATION = math.radians(6.0)
    hard_left = THETA_WALL - MAX_PENETRATION
    theta = max(hard_left, min(math.radians(180), theta))

    if dt > 0:
        theta_dot = (theta - prev_theta) / dt
        raw_ddot = (theta_dot - theta_dot_prev) / dt
        a = 0.25
        theta_ddot_f = (1 - a) * theta_ddot_f + a * raw_ddot
        theta_dot_prev = theta_dot

    if keyboard.left:
        bottle_x -= MOVE_SPEED * dt
    if keyboard.right:
        bottle_x += MOVE_SPEED * dt
    bottle_x = clamp01(bottle_x)

    thr = render_angle(theta)
    integrate_slosh(dt, thr)

    # compute flow this frame (also drives glug + pour)
    flow_rate = 0.0
    if V > 0.0:
        surf = liquid_surface(V, thr)
        lip = lip_lowest_world(thr)
        overshoot = surf - lip
        if overshoot > 0.0:
            neck_area = math.pi * R_NECK * R_NECK
            flow_rate = K_POUR * neck_area * math.sqrt(2 * G * overshoot) * 1e3
            if not paused:
                V = max(0.0, V - flow_rate * dt)

    last_tau_glug = update_glug(dt, thr, flow_rate)


def on_key_down(key):
    global V, paused, slosh_gain, glug_gain
    if key == keys.R:
        V = V_MAX
    if key == keys.SPACE:
        paused = not paused
    if key == keys.Q:
        slosh_gain = min(8.0, slosh_gain + 0.5)
    if key == keys.W:
        slosh_gain = max(0.0, slosh_gain - 0.5)
    if key == keys.E:             # '  -> more glug
        glug_gain = min(4.0, glug_gain + 0.25)
    if key == keys.R:         # ;  -> less glug
        glug_gain = max(0.0, glug_gain - 0.25)


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
last_tau_glug = 0.0


def bar_to_px(frac_x):
    margin = 120
    return margin + frac_x * (WIDTH - 2 * margin)


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


def draw():
    screen.fill((16, 18, 26))

    (m_liquid, m_total, x_com, y_com,
     tau_grav, tau_w, tau_slosh, surf, thr) = compute_state()

    tau_glug = last_tau_glug
    tau_motor = tau_grav + tau_w + tau_slosh + tau_glug

    pressing_wall = tau_w > 1e-4
    surf_tilt = VIS_TILT * slosh_gain * s_slosh * (m_liquid / (RHO * V_MAX))

    screen.draw.line((0, PIVOT_Y + 6), (WIDTH * 0.5, PIVOT_Y + 6), (70, 60, 50))

    gx = bar_to_px(GLASS_X)
    gw, gh = 70, 110
    gtop = GLASS_TOP_Y
    gbot = gtop + gh
    screen.draw.line((gx - gw / 2, gtop), (gx - gw / 2, gbot), (150, 170, 190))
    screen.draw.line((gx + gw / 2, gtop), (gx + gw / 2, gbot), (150, 170, 190))
    screen.draw.line((gx - gw / 2, gbot), (gx + gw / 2, gbot), (150, 170, 190))
    screen.draw.line((gx - gw, gbot), (gx + gw, gbot), (70, 60, 50))

    pivot_px = (bar_to_px(bottle_x), PIVOT_Y)

    wall_thr = render_angle(THETA_WALL)
    wall_px = local_to_px((0, H_TOTAL * 1.05), pivot_px, wall_thr)
    wcol = (200, 70, 70) if pressing_wall else (90, 70, 70)
    screen.draw.line(pivot_px, wall_px, wcol)
    screen.draw.text("WALL (-10 deg)", (wall_px[0] - 90, wall_px[1] - 20),
                     color=wcol, fontsize=18)

    outline = bottle_outline_points()
    outline_px = [local_to_px(p, pivot_px, thr) for p in outline]
    body_col = (70, 40, 40) if pressing_wall else (40, 55, 48)
    edge_col = (230, 90, 90) if pressing_wall else (120, 140, 130)
    pygame.draw.polygon(screen.surface, body_col, outline_px)
    pygame.draw.polygon(screen.surface, edge_col, outline_px, 2)

    if V > 0:
        boundary = bottle_outline_points()

        def wobble_h(p):
            x, y = p
            return (x * math.sin(thr) + y * math.cos(thr)
                    + surf_tilt * (x * math.cos(thr) - y * math.sin(thr)))

        wet = []
        n = len(boundary)
        for i in range(n):
            a = boundary[i]
            b = boundary[(i + 1) % n]
            ha = wobble_h(a) - surf
            hb = wobble_h(b) - surf
            a_in = ha <= 0.0
            b_in = hb <= 0.0
            if a_in:
                wet.append(a)
            if a_in != b_in:
                t = ha / (ha - hb)
                wet.append((a[0] + t * (b[0] - a[0]),
                            a[1] + t * (b[1] - a[1])))
        if len(wet) >= 3:
            wet_px = [local_to_px(p, pivot_px, thr) for p in wet]
            pygame.draw.polygon(screen.surface, (120, 30, 60), wet_px)

    com_px = local_to_px((x_com, y_com), pivot_px, thr)
    screen.draw.filled_circle(com_px, 6, (255, 85, 85))
    screen.draw.text("COM", (com_px[0] + 9, com_px[1] - 9),
                     color=(255, 150, 150), fontsize=20)
    screen.draw.filled_circle(pivot_px, 4, (210, 210, 210))

    # glug indicator: pulse a ring at the bottle mouth on each glug
    if glug_active:
        mouth_px = local_to_px((0, LIP_Y), pivot_px, thr)
        r_ring = 8 + 14 * abs(tau_glug) / max(1e-6, A_GLUG * glug_gain)
        screen.draw.circle(mouth_px, int(r_ring), (120, 180, 255))
        screen.draw.text("GLUG", (mouth_px[0] + 10, mouth_px[1] - 10),
                         color=(140, 200, 255), fontsize=18)

    panel_x = WIDTH - 270
    screen.draw.text("Motor torque  tau", (panel_x, 50),
                     color=(225, 225, 225), fontsize=22)
    cx = panel_x + 120
    y0 = 80
    screen.draw.line((cx, y0 - 6), (cx, y0 + 30), (90, 90, 90))
    blen = max(-130, min(130, tau_motor * 60.0))
    col = (255, 160, 60) if blen >= 0 else (110, 200, 255)
    if pressing_wall:
        col = (240, 70, 70)
    if abs(tau_glug) > 0.02:
        col = (120, 180, 255)
    if blen >= 0:
        screen.draw.filled_rect(Rect((cx, y0), (blen, 18)), col)
    else:
        screen.draw.filled_rect(Rect((cx + blen, y0), (-blen, 18)), col)

    fill_pct = 100.0 * V / V_MAX
    lines = [
        f"theta        : {math.degrees(theta):6.1f} deg",
        f"bottle x     : {bottle_x:6.2f}  [IMU]",
        f"volume V     : {V * 1e6:7.1f} mL ({fill_pct:5.1f}%)",
        f"liquid mass  : {m_liquid * 1000:7.1f} g",
        f"tau gravity  : {tau_grav:7.3f} N*m",
        f"tau slosh    : {tau_slosh:7.3f} N*m",
        f"tau glug     : {tau_glug:7.3f} N*m",
        f"tau MOTOR    : {tau_motor:7.3f} N*m",
        f"slosh Q/W    : {slosh_gain:4.1f}",
        f"glug  E/R    : {glug_gain:4.1f}",
    ]
    yy = 140
    for ln in lines:
        c2 = (205, 212, 225)
        if ln.startswith("tau slosh"):
            c2 = (140, 200, 255)
        if ln.startswith("tau glug"):
            c2 = (120, 180, 255)
        screen.draw.text(ln, (panel_x, yy), color=c2, fontsize=19)
        yy += 22

    screen.draw.text("D/A tilt   LEFT/RIGHT slide   Q/W slosh   E/R glug   R refill   SPACE pause",
                     (20, PIVOT_Y + 20), color=(150, 160, 175), fontsize=17)

    if glug_active:
        screen.draw.text("GLUG GLUG (opening submerged)", (20, 20),
                         color=(120, 180, 255), fontsize=24)
    elif pressing_wall:
        screen.draw.text("<< WALL: strong counter-torque", (20, 20),
                         color=(240, 90, 90), fontsize=24)

    if paused:
        screen.draw.text("PAUSED", (WIDTH // 2 - 55, 18),
                         color=(255, 220, 120), fontsize=38)