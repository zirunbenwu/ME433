# HW10 - Pinball for Pygame Zero, controlled by two buttons on a Pico
#
# Left button  -> left flipper
# Right button -> right flipper
# Space / both buttons -> launch a new ball after one drains
#
# Protocol: Pico streams "left,right\n" at ~100 Hz (1 = pressed).

import pgzrun
import math
import random
import serial
import serial.tools.list_ports

WIDTH = 500
HEIGHT = 700

# ----- Serial setup --------------------------------------------------------
SERIAL_PORT = "COM4"   # e.g. "COM3" on Windows, "/dev/tty.usbmodem1101" on Mac
BAUD = 115200


def find_pico_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        manuf = (p.manufacturer or "").lower()
        if "pico" in desc or "rp2" in desc or "raspberry" in manuf:
            return p.device
    for p in ports:
        dev = p.device.lower()
        if "usbmodem" in dev or "ttyacm" in dev or dev.startswith("com"):
            return p.device
    return None


if SERIAL_PORT is None:
    SERIAL_PORT = find_pico_port()

ser = None
if SERIAL_PORT is not None:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD, timeout=0)
        print(f"Connected to Pico on {SERIAL_PORT}")
    except Exception as e:
        print(f"Could not open {SERIAL_PORT}: {e}")
        ser = None
else:
    print("No Pico-like serial port found. Use arrow keys.")

btn_left = False
btn_right = False
serial_buf = ""


def poll_serial():
    global btn_left, btn_right, serial_buf
    if ser is None or not ser.in_waiting:
        return
    try:
        chunk = ser.read(ser.in_waiting).decode("utf-8", errors="ignore")
    except Exception:
        return
    serial_buf += chunk
    if "\n" not in serial_buf:
        return
    lines = serial_buf.split("\n")
    serial_buf = lines[-1]
    for line in reversed(lines[:-1]):
        parts = line.strip().split(",")
        if len(parts) == 2:
            try:
                btn_left = int(parts[0]) == 1
                btn_right = int(parts[1]) == 1
                return
            except ValueError:
                continue


# ----- Physics constants ---------------------------------------------------
GRAVITY = 600.0           # px / s^2
BALL_R = 10
WALL_BOUNCE = 0.55        # energy kept when hitting a wall (lower = less bouncy)
BUMPER_BOOST = 1.0        # bumpers kick the ball but no extra speed multiplier
MAX_SPEED = 1000.0

# Flipper geometry
FLIPPER_LEN = 90
FLIPPER_THICK = 14
FLIPPER_PIVOT_Y = HEIGHT - 110
LEFT_PIVOT_X = 130
RIGHT_PIVOT_X = WIDTH - 130
# Angles measured in radians, 0 = pointing right, positive = down (screen y).
LEFT_REST_ANGLE = math.radians(30)     # tilts down-right when resting
LEFT_UP_ANGLE = math.radians(-25)      # swings up
RIGHT_REST_ANGLE = math.radians(180 - 30)
RIGHT_UP_ANGLE = math.radians(180 + 25)
FLIPPER_SPEED = math.radians(900)      # rad/s while moving
FLIPPER_HIT_BOOST = 700.0              # extra speed imparted by a moving flipper

# Gutter opening between the flipper tips and the side walls
GUTTER_Y = FLIPPER_PIVOT_Y + 50

# Bumpers: (x, y, radius, points)
BUMPERS = [
    (WIDTH / 2, 240, 28, 100),
    (WIDTH / 2 - 110, 280, 24, 50),
    (WIDTH / 2 + 110, 280, 24, 50),
    (WIDTH / 2, 380, 22, 25),
]

# Wall segments forming the play-field boundary. Each entry is two endpoints.
INLANE_TOP_Y = 470
LEFT_LIP_X = LEFT_PIVOT_X - 8
RIGHT_LIP_X = RIGHT_PIVOT_X + 8

# Launch lane and top rail geometry.
TOP_WALL_Y = 70
LAUNCH_LANE_X = WIDTH - 60       # wider launch channel so the ball clearly fits
LAUNCH_LANE_TOP = 180            # y where the launch channel opens into the rail

# Curved rail at the top-right: 3 line segments approximating a quarter circle
# centered at (390, 180) with radius 110. The ball comes up the launch lane,
# rides along the inside of this arc, and exits leftward into the play field.
RAIL_PTS = [
    (390, TOP_WALL_Y),   # top: connects to the flat top wall
    (445, 85),
    (485, 125),
    (WIDTH, LAUNCH_LANE_TOP),  # bottom: meets the right edge at the launch lane top
]

WALLS = [
    # Flat top wall across the play field.
    ((0, TOP_WALL_Y), (RAIL_PTS[0][0], TOP_WALL_Y)),
    # Curved rail segments.
    (RAIL_PTS[0], RAIL_PTS[1]),
    (RAIL_PTS[1], RAIL_PTS[2]),
    (RAIL_PTS[2], RAIL_PTS[3]),
    # Left wall (vertical).
    ((0, TOP_WALL_Y), (0, INLANE_TOP_Y)),
    # Launch lane wall (vertical) -- separates launch channel from play field.
    ((LAUNCH_LANE_X, LAUNCH_LANE_TOP), (LAUNCH_LANE_X, INLANE_TOP_Y)),
    # Left inlane: funnels balls toward the left flipper.
    ((0, INLANE_TOP_Y), (LEFT_LIP_X, FLIPPER_PIVOT_Y - 6)),
    # Right inlane: funnels balls toward the right flipper.
    ((LAUNCH_LANE_X, INLANE_TOP_Y), (RIGHT_LIP_X, FLIPPER_PIVOT_Y - 6)),
    # Short vertical lips above each flipper pivot so the ball doesn't catch
    # on the pivot corner.
    ((LEFT_LIP_X, FLIPPER_PIVOT_Y - 6), (LEFT_LIP_X, FLIPPER_PIVOT_Y + 4)),
    ((RIGHT_LIP_X, FLIPPER_PIVOT_Y - 6), (RIGHT_LIP_X, FLIPPER_PIVOT_Y + 4)),
]

# ----- Game state ----------------------------------------------------------
ball_x = 0.0
ball_y = 0.0
ball_vx = 0.0
ball_vy = 0.0
ball_active = False

left_angle = LEFT_REST_ANGLE
right_angle = RIGHT_REST_ANGLE
prev_left_angle = LEFT_REST_ANGLE
prev_right_angle = RIGHT_REST_ANGLE

score = 0
balls_left = 3
game_over = False
bumper_flash = [0.0] * len(BUMPERS)   # seconds remaining of "lit" state


def launch_ball():
    """Spawn a new ball at the bottom of the launch lane (right channel)."""
    global ball_x, ball_y, ball_vx, ball_vy, ball_active
    ball_x = (LAUNCH_LANE_X + WIDTH) / 2
    ball_y = HEIGHT - 100
    # Straight up, strong enough to ride around the curved rail at the top.
    ball_vx = 0.0
    ball_vy = -1100
    ball_active = True


def reset_game():
    global score, balls_left, game_over
    score = 0
    balls_left = 3
    game_over = False
    launch_ball()


# ----- Geometry helpers ----------------------------------------------------
def flipper_endpoints(pivot_x, pivot_y, angle):
    tip_x = pivot_x + math.cos(angle) * FLIPPER_LEN
    tip_y = pivot_y + math.sin(angle) * FLIPPER_LEN
    return pivot_x, pivot_y, tip_x, tip_y


def closest_point_on_segment(px, py, ax, ay, bx, by):
    abx, aby = bx - ax, by - ay
    denom = abx * abx + aby * aby
    if denom == 0:
        return ax, ay, 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = max(0.0, min(1.0, t))
    return ax + abx * t, ay + aby * t, t


def cap_speed():
    global ball_vx, ball_vy
    s = math.hypot(ball_vx, ball_vy)
    if s > MAX_SPEED:
        ball_vx *= MAX_SPEED / s
        ball_vy *= MAX_SPEED / s


def resolve_static_segment(ax, ay, bx, by):
    """Bounce the ball off a static line segment."""
    global ball_x, ball_y, ball_vx, ball_vy
    cx, cy, _ = closest_point_on_segment(ball_x, ball_y, ax, ay, bx, by)
    dx, dy = ball_x - cx, ball_y - cy
    dist = math.hypot(dx, dy)
    radius_sum = BALL_R + 3  # wall has ~6 px visual thickness
    if dist >= radius_sum:
        return
    if dist == 0:
        nx, ny = 0.0, -1.0
    else:
        nx, ny = dx / dist, dy / dist
    overlap = radius_sum - dist
    ball_x += nx * overlap
    ball_y += ny * overlap
    vn = ball_vx * nx + ball_vy * ny
    if vn < 0:
        ball_vx -= (1 + WALL_BOUNCE) * vn * nx
        ball_vy -= (1 + WALL_BOUNCE) * vn * ny


def resolve_flipper(pivot_x, pivot_y, angle, prev_angle):
    """Push the ball off the flipper segment if overlapping, and impart speed
    based on how fast the flipper is rotating."""
    global ball_x, ball_y, ball_vx, ball_vy
    ax, ay, bx, by = flipper_endpoints(pivot_x, pivot_y, angle)
    cx, cy, t = closest_point_on_segment(ball_x, ball_y, ax, ay, bx, by)
    dx, dy = ball_x - cx, ball_y - cy
    dist = math.hypot(dx, dy)
    radius_sum = BALL_R + FLIPPER_THICK / 2
    if dist >= radius_sum:
        return False
    if dist == 0:
        nx, ny = 0.0, -1.0
    else:
        nx, ny = dx / dist, dy / dist
    # Push the ball out of penetration.
    overlap = radius_sum - dist
    ball_x += nx * overlap
    ball_y += ny * overlap
    # Reflect velocity around the normal.
    vn = ball_vx * nx + ball_vy * ny
    if vn < 0:
        ball_vx -= 2 * vn * nx
        ball_vy -= 2 * vn * ny
    # Add a boost if the flipper is actively swinging up.
    dangle = angle - prev_angle
    if dangle != 0:
        # Tangential velocity at the contact point on the flipper.
        rx = cx - pivot_x
        ry = cy - pivot_y
        # omega = dangle / dt, but we just want a kick proportional to motion.
        tangential_x = -ry * dangle * 60   # 60 approx for per-frame scaling
        tangential_y = rx * dangle * 60
        # Project that onto the surface normal and add as a boost.
        boost = (tangential_x * nx + tangential_y * ny)
        if boost > 0:
            ball_vx += nx * boost * 0.4
            ball_vy += ny * boost * 0.4
    return True


# ----- pgz callbacks -------------------------------------------------------
def update(dt):
    global ball_x, ball_y, ball_vx, ball_vy, ball_active
    global left_angle, right_angle, prev_left_angle, prev_right_angle
    global score, balls_left, game_over

    poll_serial()

    left = btn_left
    right = btn_right
    launch = btn_left and btn_right

    # Flipper angles ease toward their target.
    prev_left_angle = left_angle
    prev_right_angle = right_angle
    target_left = LEFT_UP_ANGLE if left else LEFT_REST_ANGLE
    target_right = RIGHT_UP_ANGLE if right else RIGHT_REST_ANGLE
    step = FLIPPER_SPEED * dt
    if left_angle < target_left:
        left_angle = min(left_angle + step, target_left)
    elif left_angle > target_left:
        left_angle = max(left_angle - step, target_left)
    # Right flipper: "up" is a larger angle (180+25), "rest" is smaller (180-30).
    if right_angle < target_right:
        right_angle = min(right_angle + step, target_right)
    elif right_angle > target_right:
        right_angle = max(right_angle - step, target_right)

    if game_over:
        if launch:
            reset_game()
        return

    if not ball_active:
        if launch:
            launch_ball()
        return

    # Integrate motion with gravity.
    ball_vy += GRAVITY * dt
    ball_x += ball_vx * dt
    ball_y += ball_vy * dt
    cap_speed()

    # Side walls.
    if ball_x < BALL_R:
        ball_x = BALL_R
        ball_vx = -ball_vx * WALL_BOUNCE
    if ball_x > WIDTH - BALL_R:
        ball_x = WIDTH - BALL_R
        ball_vx = -ball_vx * WALL_BOUNCE
    # Ceiling.
    if ball_y < BALL_R:
        ball_y = BALL_R
        ball_vy = -ball_vy * WALL_BOUNCE

    # Bumpers.
    for i, (bx, by, br, pts) in enumerate(BUMPERS):
        dx = ball_x - bx
        dy = ball_y - by
        d = math.hypot(dx, dy)
        if d < br + BALL_R:
            if d == 0:
                nx, ny = 0.0, -1.0
            else:
                nx, ny = dx / d, dy / d
            ball_x = bx + nx * (br + BALL_R)
            ball_y = by + ny * (br + BALL_R)
            vn = ball_vx * nx + ball_vy * ny
            if vn < 0:
                ball_vx -= (1 + WALL_BOUNCE) * vn * nx
                ball_vy -= (1 + WALL_BOUNCE) * vn * ny
            # Gentle outward kick.
            ball_vx += nx * 80
            ball_vy += ny * 80
            ball_vx *= BUMPER_BOOST
            ball_vy *= BUMPER_BOOST
            score += pts
            bumper_flash[i] = 0.15

    for i in range(len(bumper_flash)):
        bumper_flash[i] = max(0.0, bumper_flash[i] - dt)

    # Inlane walls.
    for (ax, ay), (bx, by) in WALLS:
        resolve_static_segment(ax, ay, bx, by)

    # Flippers.
    resolve_flipper(LEFT_PIVOT_X, FLIPPER_PIVOT_Y, left_angle, prev_left_angle)
    resolve_flipper(RIGHT_PIVOT_X, FLIPPER_PIVOT_Y, right_angle, prev_right_angle)

    # Drain: ball falls past the bottom.
    if ball_y > HEIGHT + 20:
        ball_active = False
        balls_left -= 1
        if balls_left <= 0:
            game_over = True


def draw():
    screen.fill((15, 12, 30))

    # Play-field decoration.
    screen.draw.line((0, 0), (0, HEIGHT), (60, 60, 100))
    screen.draw.line((WIDTH - 1, 0), (WIDTH - 1, HEIGHT), (60, 60, 100))

    # Inlane walls (drawn thick so the ball visibly hits something).
    for (ax, ay), (bx, by) in WALLS:
        steps = max(2, int(math.hypot(bx - ax, by - ay) / 6))
        for k in range(steps + 1):
            t = k / steps
            screen.draw.filled_circle(
                (ax + (bx - ax) * t, ay + (by - ay) * t),
                3, (130, 130, 180))

    # Bumpers.
    for i, (bx, by, br, pts) in enumerate(BUMPERS):
        base = (200, 80, 200) if bumper_flash[i] > 0 else (110, 60, 160)
        screen.draw.filled_circle((bx, by), br, base)
        screen.draw.filled_circle((bx, by), br - 6, (230, 200, 240))
        screen.draw.text(str(pts), center=(bx, by),
                         color=(40, 20, 60), fontsize=20)

    # Flippers as fat lines.
    for pivot_x, angle, color in (
        (LEFT_PIVOT_X, left_angle, (240, 200, 90)),
        (RIGHT_PIVOT_X, right_angle, (240, 200, 90)),
    ):
        ax, ay, bx, by = flipper_endpoints(pivot_x, FLIPPER_PIVOT_Y, angle)
        # Thick line drawn as a series of circles for rounded ends.
        steps = 12
        for k in range(steps + 1):
            t = k / steps
            x = ax + (bx - ax) * t
            y = ay + (by - ay) * t
            screen.draw.filled_circle((x, y), FLIPPER_THICK / 2, color)
        screen.draw.filled_circle((pivot_x, FLIPPER_PIVOT_Y), 6, (60, 40, 20))

    # Ball.
    if ball_active:
        screen.draw.filled_circle((ball_x, ball_y), BALL_R, (240, 240, 255))
        screen.draw.filled_circle((ball_x - 3, ball_y - 3), 3, (255, 255, 255))

    # HUD.
    screen.draw.text(f"Score: {score}", (12, 10), color="white", fontsize=30)
    screen.draw.text(f"Balls: {balls_left}", (12, 42), color="white", fontsize=30)

    lc = (120, 220, 120) if btn_left else (70, 70, 70)
    rc = (120, 220, 120) if btn_right else (70, 70, 70)
    screen.draw.filled_circle((WIDTH - 60, 28), 11, lc)
    screen.draw.filled_circle((WIDTH - 28, 28), 11, rc)
    screen.draw.text("L   R", (WIDTH - 78, 44), color="white", fontsize=18)

    status = ("Pico connected" if ser is not None
              else "Pico not connected")
    screen.draw.text(status, (12, HEIGHT - 24),
                     color=(180, 180, 180), fontsize=18)

    if game_over:
        screen.draw.text("GAME OVER",
                         center=(WIDTH / 2, HEIGHT / 2 - 20),
                         color="white", fontsize=60)
        screen.draw.text(f"Final score: {score}",
                         center=(WIDTH / 2, HEIGHT / 2 + 30),
                         color="white", fontsize=32)
        screen.draw.text("Press both buttons to restart",
                         center=(WIDTH / 2, HEIGHT / 2 + 80),
                         color=(200, 200, 200), fontsize=22)
    elif not ball_active:
        screen.draw.text("Press both buttons to launch",
                         center=(WIDTH / 2, HEIGHT / 2),
                         color=(220, 220, 220), fontsize=24)


pgzrun.go()