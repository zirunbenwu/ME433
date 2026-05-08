import matplotlib.pyplot as plt
import numpy as np
import csv

for name in ['sigA', 'sigB', 'sigC', 'sigD']:
    # read CSV into time and value lists
    t = []
    data = []
    with open(name + '.csv', 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                t.append(float(row[0]))
                data.append(float(row[1]))
            except ValueError:
                continue  # skip header row if present
    t = np.array(t)
    data = np.array(data)

    # sample rate = number of data points / total time of samples
    Fs = len(t) / t[-1]

    y = data           # the data to make the fft from
    n = len(y)         # length of the signal
    k = np.arange(n)
    T = n / Fs
    frq = k / T                       # two sides frequency range
    frq = frq[range(int(n/2))]        # one side frequency range
    Y = np.fft.fft(y) / n             # fft computing and normalization
    Y = Y[range(int(n/2))]

    fig, (ax1, ax2) = plt.subplots(2, 1)
    fig.suptitle(name + '  (Fs = ' + str(round(Fs, 2)) + ' Hz)')
    ax1.plot(t, y, 'b')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Amplitude')
    ax2.loglog(frq, abs(Y), 'b')      # plotting the fft
    ax2.set_xlabel('Freq (Hz)')
    ax2.set_ylabel('|Y(freq)|')
    plt.tight_layout()

plt.show()