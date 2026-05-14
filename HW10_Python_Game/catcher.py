# HW10 - Catcher game for Pygame Zero, controlled by two buttons on a Pico
#
# The Pico streams "left,right\n" lines at ~100 Hz (1 = pressed). We read the
# most recent complete line each frame and move the paddle accordingly.
# Arrow keys also work as a fallback if no Pico is connected.
#
# Run from VSCode (this file calls pgzrun.go() at the bottom), or from the
# terminal with:  pgzrun catcher.py
 
import pgzrun
import random
import serial
import serial.tools.list_ports
 
WIDTH = 800
HEIGHT = 600
 
# ----- Serial setup --------------------------------------------------------
# Set SERIAL_PORT explicitly if auto-detect picks the wrong device.
# Mac:     "/dev/tty.usbmodemXXXX"
# Linux:   "/dev/ttyACM0"
# Windows: "COM3" (check Device Manager)
SERIAL_PORT = "COM4"
BAUD = 115200
 
 
def find_pico_port():
    """Best-effort guess at which serial port the Pico is on."""
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        manuf = (p.manufacturer or "").lower()
        if "pico" in desc or "rp2" in desc or "raspberry" in manuf:
            return p.device
    # Fall back to the first USB-CDC-looking port.
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
 
# ----- Game state ----------------------------------------------------------
PLAYER_W = 90
PLAYER_H = 18
PLAYER_Y = HEIGHT - 50
PLAYER_SPEED = 7
 
player_x = WIDTH / 2
btn_left = False
btn_right = False
 
items = []          # each: {"x", "y", "vy", "type"}
spawn_timer = 0.0
score = 0
lives = 3
game_over = False
serial_buf = ""
 
 
def reset_game():
    global items, score, lives, game_over, spawn_timer, player_x
    items = []
    score = 0
    lives = 3
    game_over = False
    spawn_timer = 0.0
    player_x = WIDTH / 2
 
 
# ----- Serial reading ------------------------------------------------------
def poll_serial():
    """Drain the serial buffer; keep only the latest complete line's state."""
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
    serial_buf = lines[-1]  # incomplete tail saved for next poll
    # Walk from newest to oldest, take the first well-formed line.
    for line in reversed(lines[:-1]):
        parts = line.strip().split(",")
        if len(parts) == 2:
            try:
                btn_left = int(parts[0]) == 1
                btn_right = int(parts[1]) == 1
                return
            except ValueError:
                continue
 
 
# ----- pgz callbacks -------------------------------------------------------
def update(dt):
    global player_x, spawn_timer, score, lives, game_over
 
    poll_serial()
 
    # Keyboard fallback so the game is playable without the Pico.
    left = btn_left or keyboard.left
    right = btn_right or keyboard.right
 
    if game_over:
        # Press BOTH buttons (or space) to restart.
        if (btn_left and btn_right) or keyboard.space:
            reset_game()
        return
 
    if left:
        player_x -= PLAYER_SPEED
    if right:
        player_x += PLAYER_SPEED
    player_x = max(PLAYER_W / 2, min(WIDTH - PLAYER_W / 2, player_x))
 
    # Spawn items; rate increases with score.
    spawn_timer += dt
    spawn_interval = max(0.3, 1.1 - score * 0.004)
    if spawn_timer >= spawn_interval:
        spawn_timer = 0.0
        kind = random.choices(["good", "bad"], weights=[3, 2])[0]
        items.append({
            "x": random.uniform(20, WIDTH - 20),
            "y": -20,
            "vy": random.uniform(200, 340),
            "type": kind,
        })
 
    # Move items.
    for it in items:
        it["y"] += it["vy"] * dt
 
    # Resolve collisions with paddle and offscreen items.
    keep = []
    paddle_top = PLAYER_Y - PLAYER_H / 2
    paddle_bot = PLAYER_Y + PLAYER_H / 2
    for it in items:
        within_y = paddle_top - 12 < it["y"] < paddle_bot + 12
        within_x = abs(it["x"] - player_x) < PLAYER_W / 2 + 12
        if within_y and within_x:
            if it["type"] == "good":
                score += 10
            else:
                lives -= 1
                if lives <= 0:
                    game_over = True
        elif it["y"] < HEIGHT + 20:
            keep.append(it)
    items[:] = keep
 
 
def draw():
    screen.fill((20, 24, 40))
 
    # Paddle.
    paddle_rect = Rect(player_x - PLAYER_W / 2,
                       PLAYER_Y - PLAYER_H / 2,
                       PLAYER_W, PLAYER_H)
    screen.draw.filled_rect(paddle_rect, (200, 220, 255))
 
    # Falling items.
    for it in items:
        color = (90, 220, 120) if it["type"] == "good" else (240, 80, 80)
        screen.draw.filled_circle((it["x"], it["y"]), 12, color)
 
    # HUD.
    screen.draw.text(f"Score: {score}", (12, 10), color="white", fontsize=30)
    screen.draw.text(f"Lives: {lives}", (12, 42), color="white", fontsize=30)
 
    # Button indicator lights.
    lc = (120, 220, 120) if btn_left else (70, 70, 70)
    rc = (120, 220, 120) if btn_right else (70, 70, 70)
    screen.draw.filled_circle((WIDTH - 60, 28), 11, lc)
    screen.draw.filled_circle((WIDTH - 28, 28), 11, rc)
    screen.draw.text("L   R", (WIDTH - 78, 44), color="white", fontsize=18)
 
    # Connection status.
    status = ("Pico connected" if ser is not None
              else "Pico not connected - using arrow keys")
    screen.draw.text(status, (12, HEIGHT - 24), color=(180, 180, 180), fontsize=18)
 
    if game_over:
        screen.draw.text("GAME OVER",
                         center=(WIDTH / 2, HEIGHT / 2 - 20),
                         color="white", fontsize=72)
        screen.draw.text(f"Final score: {score}",
                         center=(WIDTH / 2, HEIGHT / 2 + 30),
                         color="white", fontsize=32)
        screen.draw.text("Press both buttons (or space) to restart",
                         center=(WIDTH / 2, HEIGHT / 2 + 80),
                         color=(200, 200, 200), fontsize=24)
 
 
pgzrun.go()