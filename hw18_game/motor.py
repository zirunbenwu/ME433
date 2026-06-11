# motor_tuner.py
# Phase 1: tune the current loop (ITEST + plot), test the buzz.
# Press 'z' to close the serial port and launch the game (game2.py).
#
#   a              read current (mA)
#   f <duty>       set open-loop duty (turn motor on)
#   g <kp> <ki>    set current gains
#   h              get current gains
#   k              run ITEST current step + plot
#   y              read encoder angle (deg)
#   zero           zero the encoder
#   b              continuous buzz on (bench feel)
#   v <amp> <freq> set buzz amplitude (mA) and frequency (Hz)
#   B <amp> <dur>  one buzz burst (mA, ms)
#   p              stop motor
#   z              LAUNCH GAME (closes serial)
#   q              quit
#
# Needs:  pip install pyserial matplotlib
# Run:    python motor_tuner.py

import sys
import time
import subprocess
import serial
from statistics import mean
import matplotlib.pyplot as plt

# ---- match your firmware ----
PORT      = 'COM13'
BAUD      = 115200
GAME_FILE = 'game2.py'        # path to the pgzero game

cur_kp = 0.0
cur_ki = 0.0

ser = serial.Serial(PORT, BAUD, timeout=1)
print('Opening', ser.name)
time.sleep(2.0)
ser.reset_input_buffer()


def read_value_line(timeout_s=3.0):
    """Read a single-value reply, skipping the streaming 'angle,current' lines
    (which contain a comma) and any '#' comment lines."""
    t0 = time.time()
    while time.time() - t0 < timeout_s:
        ln = ser.read_until(b'\n').decode('utf-8', errors='ignore').strip()
        if ln == '' or ln.startswith('#'):
            continue
        if ',' in ln:           # stream line "angle,current" - not a reply
            continue
        return ln
    return None


def plot_itest():
    print('Running ITEST...')
    # drain whatever is buffered, then send k
    time.sleep(0.1)
    ser.reset_input_buffer()
    time.sleep(0.05)
    ser.write(b'k\n')
    ser.flush()

    # scan for the "# ITEST N" header among the streaming lines
    n = None
    t0 = time.time()
    while time.time() - t0 < 12.0:
        ln = ser.read_until(b'\n').decode('utf-8', errors='ignore').strip()
        if ln.startswith('# ITEST'):
            try:
                n = int(ln.split()[-1])
            except (ValueError, IndexError):
                n = None
            break
        # ignore everything else (stream lines, other comments)
    if not n:
        print('  no ITEST header received - try again')
        return

    # collect exactly n "des act" pairs; skip any stream lines that sneak in
    des, act = [], []
    t0 = time.time()
    while len(des) < n and time.time() - t0 < 12.0:
        ln = ser.read_until(b'\n').decode('utf-8', errors='ignore').strip()
        if ln == '' or ln.startswith('#'):
            continue
        if ',' in ln:            # stream "angle,current" - not ITEST data
            continue
        parts = ln.split()
        if len(parts) == 2:
            try:
                des.append(float(parts[0]))
                act.append(float(parts[1]))
            except ValueError:
                pass
    if len(des) < n:
        print(f'  only got {len(des)}/{n} samples - try again')
        return

    score = mean(abs(d - a) for d, a in zip(des, act))
    print(f'  {len(des)} samples, score={score:.2f}')
    plt.figure()
    plt.plot(des, 'r*-', label='desired', markersize=3)
    plt.plot(act, 'b*-', label='actual', markersize=3)
    plt.title(f'ITEST  Kp={cur_kp}  Ki={cur_ki}  score={score:.2f}')
    plt.xlabel('sample'); plt.ylabel('current (mA)')
    plt.legend(); plt.grid(True, linestyle='--', alpha=0.5)
    plt.tight_layout(); plt.show()


def launch_game():
    print('Closing serial and launching game...')
    try:
        ser.write(b'q\n')          # stop motor
    except Exception:
        pass
    ser.close()                    # free the port for the game
    time.sleep(0.5)
    subprocess.run([sys.executable, '-m', 'pgzero', GAME_FILE])
    print('Game closed.')


print("""
=== CURRENT-LOOP TUNER ===
  a            read current (mA)
  f <duty>     set duty (motor on)
  g <kp> <ki>  set current gains
  h            get current gains
  k            run ITEST + plot
  y            read encoder angle
  zero         zero the encoder
  b            continuous buzz (feel)
  v <amp> <freq>  set buzz amp(mA)/freq(Hz)
  B <amp> <dur>   one buzz burst (mA, ms)
  p            stop motor
  z            LAUNCH GAME (closes serial)
  q            quit
""")

while True:
    try:
        raw = input('> ').strip()
    except (EOFError, KeyboardInterrupt):
        break
    if raw == '':
        continue
    parts = raw.split()
    cmd = parts[0]

    if cmd == 'z':
        launch_game()
        break

    if cmd == 'q':
        ser.write(b'q\n')
        ser.close()
        print('bye')
        break

    if cmd == 'a':
        ser.reset_input_buffer(); ser.write(b'a\n')
        print('current =', read_value_line(), 'mA')

    elif cmd == 'f':
        if len(parts) == 2:
            ser.write(f'f {parts[1]}\n'.encode()); print('duty set', parts[1])
        else:
            print('usage: f <duty>')

    elif cmd == 'g':
        if len(parts) == 3:
            cur_kp, cur_ki = float(parts[1]), float(parts[2])
            ser.write(f'g {parts[1]} {parts[2]}\n'.encode())
            print('gains set Kp', cur_kp, 'Ki', cur_ki)
        else:
            print('usage: g <kp> <ki>')

    elif cmd == 'h':
        ser.reset_input_buffer(); ser.write(b'h\n')
        print('gains:', read_value_line())

    elif cmd == 'k':
        plot_itest()

    elif cmd == 'y':
        ser.reset_input_buffer(); ser.write(b'y\n')
        print('angle =', read_value_line(), 'deg')

    elif cmd == 'zero':
        ser.write(b'z\n'); print('encoder zeroed')

    elif cmd == 'b':
        ser.write(b'b\n'); print('buzz on (p to stop)')

    elif cmd == 'v':
        if len(parts) == 3:
            ser.write(f'v {parts[1]} {parts[2]}\n'.encode())
            print('buzz amp/freq set', parts[1], parts[2])
        else:
            print('usage: v <amp_mA> <freq_Hz>')

    elif cmd == 'B':
        if len(parts) == 3:
            ser.write(f'B {parts[1]} {parts[2]}\n'.encode())
            print('buzz burst', parts[1], 'mA for', parts[2], 'ms')
        else:
            print('usage: B <amp_mA> <dur_ms>')

    elif cmd == 'p':
        ser.write(b'p\n'); print('stop')

    else:
        print('unknown:', raw)

try:
    ser.write(b'q\n')
    ser.close()
except Exception:
    pass
print('Closed.')