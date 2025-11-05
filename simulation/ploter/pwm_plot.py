import numpy as np
import matplotlib.pyplot as plt

# Settings
T = 1  # Period in milliseconds
f = 1 / T  # Frequency in kHz
time = np.linspace(0, 5 * T, 1000)

# Duty cycles to show
duty_cycles = [0.25, 0.5, 0.75]

fig, axs = plt.subplots(len(duty_cycles), 1, figsize=(10, 6), sharex=True)
fig.suptitle('PWM Signals with Varying Duty Cycles', fontsize=16)

for i, D in enumerate(duty_cycles):
    pwm_signal = ((time % T) < (D * T)).astype(float)
    axs[i].plot(time, pwm_signal, drawstyle='steps-post')
    axs[i].set_ylim(-0.2, 1.2)
    axs[i].set_ylabel(f'{int(D*100)}% Duty')
    axs[i].grid(True)

axs[-1].set_xlabel('Time (ms)')
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
