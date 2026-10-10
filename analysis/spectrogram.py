import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import ShortTimeFFT
from scipy.signal import butter
from scipy.signal import filtfilt

from scipy.signal.windows import gaussian
import pandas as pd

table = pd.read_csv("data/run_up_on_track_no_cab.csv")



# Setup signal, window parameters, compute SFT.stft(), and plot via plt.imshow() using SFT.extent()
fs = 500 
unfilt_rpm = np.array(table.unfilt_rpm)
b,a = butter(4,20,'high',fs=fs)
filtsig = filtfilt(b,a,unfilt_rpm)

SFT = ShortTimeFFT(gaussian(1000, std=int(0.05 * fs), sym=True), hop=10, fs=fs)
spectrogram_data = np.abs(SFT.stft(filtsig))

plt.figure(figsize=(10, 4))
plt.imshow(spectrogram_data, origin='lower', aspect='auto', extent=SFT.extent(len(unfilt_rpm)), cmap='inferno')
plt.plot(np.array(table.Timestamp),unfilt_rpm/60,linewidth=1,alpha=0.2,color="white")
plt.title('RPM Noise')
# plt.figure(figsize=(10, 4))
# plt.imshow(spectrogram_vco_data, origin='lower', aspect='auto', extent=SFT.extent(len(unfilt_rpm)), cmap='inferno')
# plt.title('RPM Noise + Once Per Revolution Reference')
# plt.figure(figsize=(10, 4))
# plt.plot(np.array(table.Timestamp),filtsig)
# plt.figure(figsize=(10, 4))
# plt.plot(np.array(table.Timestamp),unfilt_rpm)
plt.show()

