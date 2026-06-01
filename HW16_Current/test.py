import serial
import matplotlib.pyplot as plt

# ---- CONFIG ----
PORT = 'COM10'       
BAUD = 115200
# ----------------

# Open the serial port
ser = serial.Serial(PORT, BAUD, timeout=2)
ser.reset_input_buffer()        # clear any stale data
print(f"Opened {PORT} at {BAUD}")

# Send the trigger character
ser.write(b'a')
print("Sent 'a', collecting data...")

# Collect data until we see "done" (or 400 samples)
indices = []
desired = []
actual = []

while True:
    line = ser.readline().decode('utf-8', errors='ignore').strip()
    if line == "done":
        break
    if line == "":
        if len(indices) >= 400:
            break
        continue
    parts = line.split()
    if len(parts) != 3:
        continue
    try:
        indices.append(int(parts[0]))
        desired.append(int(parts[1]))
        actual.append(int(parts[2]))
    except ValueError:
        continue
    if len(indices) >= 400:
        break

ser.close()
print(f"Collected {len(indices)} samples")

# Plot
plt.figure(figsize=(11, 6))
plt.plot(indices, desired, 'r--', label='Desired current', linewidth=2)
plt.plot(indices, actual, 'b-', label='Actual current', linewidth=1)
plt.xlabel('Sample (1 ms each)')
plt.ylabel('Current (units of 1/3 mA)')
plt.title('PI Current Controller (kp=0.4, ki=0.08)')
plt.legend()
plt.grid(True, alpha=0.3)
plt.tight_layout()

# Save the plot image
plt.savefig('controller_plot.png', dpi=150)
print("Saved controller_plot.png")

# Show it on screen too
plt.show()