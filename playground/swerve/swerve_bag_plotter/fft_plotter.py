import numpy as np
from matplotlib import pyplot as plt
from state_loader import get_states

# data = get_states("data/diffyjr_2022-06-08-23-52-28_0.json")
# data = get_states("data/diffyjr_2022-06-09-22-42-13_0.json")  # perimeter
# data = get_states("data/diffyjr_2022-06-09-23-07-18_0.json")  # reference recorded (wrong units)
# data = get_states("data/diffyjr_2022-06-09-23-41-58_0.json")  # figure 8's

# no angle controller
# data = get_states("data/diffyjr_2022-06-09-23-55-06_0.json")  # 10.5 V
# data = get_states("data/diffyjr_2022-06-11-00-38-10_0.json")  # 10.5 V
# data = get_states("data/diffyjr_2022-06-11-00-44-19_0.json")  # 11.0 V
# data = get_states("data/diffyjr_2022-06-11-00-46-46_0.json")  # 12.0 V

# no voltage recorded
# data = get_states("data/diffyjr_2022-06-11-12-54-00_0.json")
# data = get_states("data/diffyjr_2022-06-11-14-31-53_0.json")
# data = get_states("data/diffyjr_2022-06-11-14-34-45_0.json")
data = get_states("data/diffyjr_2022-06-11-14-39-19_0.json")

length = min([len(data[module_index]) for module_index in range(len(data))])

module_index = 0
reference_voltage = []
measured_voltage = []
measured_azimuth = []
reference_azimuth = []
timestamps = []
for index in range(length):
    # for module_index in range(len(data)):
    module_data = data[module_index][index]
    reference_voltage.append(module_data["hi_voltage_ref"])
    measured_voltage.append(module_data["hi_voltage"])
    measured_azimuth.append(module_data["azimuth"])
    reference_azimuth.append(module_data["azimuth_ref"])
    timestamps.append(module_data["time"])

reference_voltage = np.array(reference_voltage)
measured_voltage = np.array(measured_voltage)
measured_azimuth = np.array(measured_azimuth)
reference_azimuth = np.array(reference_azimuth)
timestamps = np.array(timestamps)

def plot_fft(signal, label):
    freqs = np.fft.fft(signal)
    shift_freqs = np.fft.fftshift(freqs)
    log_freqs = np.log10(np.abs(shift_freqs))
    plt.plot(log_freqs, label=label)

measured_azimuth_aligned = np.append(measured_azimuth[3:], [0, 0, 0])

# reference_voltage_rms = np.sqrt(np.mean(reference_voltage**2))
# measured_azimuth_rms = np.sqrt(np.mean(measured_azimuth**2))

# plt.figure(1)
# plot_fft(reference_voltage / reference_voltage_rms, "reference_voltage")
# plot_fft(measured_azimuth / measured_azimuth_rms, "azimuth")

# print(reference_azimuth)
plt.figure(1)
plt.plot(timestamps, reference_azimuth, label="reference_azimuth")
plt.plot(timestamps, measured_azimuth, label="azimuth")
plt.plot(timestamps, measured_azimuth_aligned, label="azimuth aligned")
plt.legend()
plt.figure(2)
correlated = np.correlate(measured_azimuth, reference_azimuth, "full")
plt.plot(correlated, label="correlated")

plt.figure(3)
ifft_corr = np.fft.ifftshift(correlated)
plt.plot(ifft_corr, label="correlated")
print(np.argmax(ifft_corr))

plt.legend()
plt.show()