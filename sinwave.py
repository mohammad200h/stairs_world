import numpy as np
import matplotlib.pyplot as plt

# 1. Define the parameters of your sine wave
frequency = 1  # Frequency of the sine wave in Hz
amplitude = 1  # Amplitude of the sine wave
sampling_rate = 100  # Number of samples per second
duration = 1 # Duration of the sine wave in seconds

# 2. Create the time array
time = np.linspace(0, duration, int(sampling_rate * duration), endpoint=False)

# 3. Generate the sine wave data
sine_wave = amplitude * np.sin(2 * np.pi * frequency * time)

# 4. (Optional) Plot the sine wave using Matplotlib
plt.plot(time, sine_wave)
plt.xlabel("Time (s)")
plt.ylabel("Amplitude")
plt.title(f"Sine Wave: Frequency={frequency} Hz, Amplitude={amplitude}")
plt.grid(True)
plt.show()
