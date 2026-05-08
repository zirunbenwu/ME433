import matplotlib.pyplot as plt
import numpy as np
import csv
 
# X (window size) chosen by eye after trying several values per signal
X_values = {
    'sigA': 50,   # smooths fast noise; tracks the ~2 Hz oscillation
    'sigB': 30,   # cuts the noise but keeps the medium-frequency wobble
    'sigC':  5,   # square wave: small X preserves the sharp edges
    'sigD': 15,   # cleans up noise without rounding the sawtooth edges
}
 
def load_csv(path):
    t, y = [], []
    with open(path, 'r') as f:
        for row in csv.reader(f):
            if not row:
                continue
            try:
                t.append(float(row[0]))
                y.append(float(row[1]))
            except ValueError:
                continue   # skip header row if present
    return np.array(t), np.array(y)
 
def moving_average(data, X):
    # average the last X points; assume the X points before the start were 0
    out = np.zeros(len(data))
    s = 0.0
    for i in range(len(data)):
        s += data[i]
        if i >= X:
            s -= data[i - X]
        out[i] = s / X
    return out
 
def fft_one_sided(y, Fs):
    n = len(y)
    k = np.arange(n)
    T = n / Fs
    frq = k / T                    # two-sided frequency range
    frq = frq[range(int(n/2))]     # one-sided
    Y = np.fft.fft(y) / n          # FFT, normalized
    Y = Y[range(int(n/2))]
    return frq, Y
 
for name in ['sigA', 'sigB', 'sigC', 'sigD']:
    t, data = load_csv(f'{name}.csv')
    Fs = len(t) / t[-1]               # sample rate
 
    X = X_values[name]
    filtered = moving_average(data, X)
 
    frq, Y_unf = fft_one_sided(data, Fs)
    _,   Y_fil = fft_one_sided(filtered, Fs)
 
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    fig.suptitle(f'{name}  -  Moving Average,  X = {X}   (Fs = {Fs:.1f} Hz)')
 
    ax1.plot(t, data,     'k', linewidth=0.8, label='unfiltered')
    ax1.plot(t, filtered, 'r', linewidth=1.2, label='filtered')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Amplitude')
    ax1.legend(loc='upper right')
 
    ax2.loglog(frq, abs(Y_unf), 'k', linewidth=0.8, label='unfiltered')
    ax2.loglog(frq, abs(Y_fil), 'r', linewidth=1.0, label='filtered')
    ax2.set_xlabel('Freq (Hz)')
    ax2.set_ylabel('|Y(freq)|')
    ax2.legend(loc='lower left')
 
    plt.tight_layout()
 
plt.show()