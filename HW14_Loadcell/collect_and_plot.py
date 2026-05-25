import sys
import time
import numpy as np
import matplotlib.pyplot as plt
import serial


def collect(port: str, n_samples: int, baud: int = 115200):
    with serial.Serial(port, baud, timeout=2) as ser:

        time.sleep(2.0)
        ser.reset_input_buffer()
        ser.write(f"{n_samples}\n".encode())
        ser.flush()


        deadline = time.time() + max(30.0, n_samples / 80.0 + 10.0)
        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if line.startswith("BEGIN"):
                break
            if time.time() > deadline:
                raise RuntimeError("Timed out waiting for BEGIN from Pico")

        raws, filts, ts = [], [], []
        for _ in range(n_samples):
            line = ser.readline().decode(errors="ignore").strip()
            if not line or line == "END":
                break
            parts = line.split(",")
            if len(parts) != 3:
                continue
            raws.append(int(parts[0]))
            filts.append(float(parts[1]))
            ts.append(int(parts[2]))

        # Drain the END line if we didn't already consume it.
        ser.readline()

    return (np.array(ts, dtype=float),
            np.array(raws, dtype=float),
            np.array(filts, dtype=float))


def plot(t_ms: np.ndarray, raw: np.ndarray, filt: np.ndarray) -> None:
    t = t_ms / 1000.0
    fs = (len(t) - 1) / (t[-1] - t[0])
    print(f"Captured {len(t)} samples; effective fs = {fs:.2f} Hz "
          f"(Nyquist = {fs/2:.2f} Hz)")

    np.savetxt("hx711_data.csv",
               np.column_stack([t_ms, raw, filt]),
               delimiter=",",
               header="t_ms,raw,filtered",
               comments="",
               fmt=["%d", "%d", "%.3f"])
    print("Saved raw data to hx711_data.csv")

    # Subtract DC so the FFT plot isn't dominated by the bias offset.
    raw_ac  = raw  - np.mean(raw)
    filt_ac = filt - np.mean(filt)

    N = len(raw_ac)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)
    raw_fft  = np.abs(np.fft.rfft(raw_ac))  / N
    filt_fft = np.abs(np.fft.rfft(filt_ac)) / N

    fig, (ax_t, ax_f) = plt.subplots(2, 1, figsize=(10, 8))

    ax_t.plot(t, raw,  label="Raw",          alpha=0.5)
    ax_t.plot(t, filt, label="IIR filtered", linewidth=1.6)
    ax_t.set_xlabel("Time (s)")
    ax_t.set_ylabel("HX711 reading (counts)")
    ax_t.set_title(f"HX711 force sensor - fs ≈ {fs:.1f} Hz")
    ax_t.grid(True, alpha=0.3)
    ax_t.legend()

    ax_f.semilogy(freqs, raw_fft,  label="Raw",          alpha=0.6)
    ax_f.semilogy(freqs, filt_fft, label="IIR filtered", linewidth=1.6)
    ax_f.set_xlabel("Frequency (Hz)")
    ax_f.set_ylabel("Magnitude")
    ax_f.set_title("FFT of raw vs filtered signal")
    ax_f.set_xlim(0, fs / 2)
    ax_f.grid(True, alpha=0.3, which="both")
    ax_f.legend()

    plt.tight_layout()
    plt.savefig("hx711_data.png", dpi=120)
    print("Saved plot to hx711_data.png")
    plt.show()


def main():
    if len(sys.argv) < 2:
        print("Usage: python collect_and_plot.py <serial_port> [n_samples]")
        sys.exit(1)

    port = sys.argv[1]
    n    = int(sys.argv[2]) if len(sys.argv) > 2 else 1000

    t_ms, raw, filt = collect(port, n)
    if len(t_ms) < 2:
        print("Not enough samples returned; check wiring / serial port.")
        sys.exit(1)
    plot(t_ms, raw, filt)


if __name__ == "__main__":
    main()
