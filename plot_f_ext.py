import os
import numpy as np
import matplotlib.pyplot as plt

def filenames(startwith):
    for filename in os.listdir('data/plot_f_ext'):
        if filename.startswith(startwith):
            yield filename

def load_datasets(startwith):
    return [
        np.loadtxt('data/plot_f_ext/'+filename, delimiter=',')
        for filename in filenames(startwith)
    ]

data_v2 = load_datasets('f_ext_v2_success')
data_v3 = load_datasets('f_ext_v3_success')
data_v4 = load_datasets('f_ext_v4_success')

# fig, ax = plt.subplots(3, 2, sharey=True, layout='constrained')

# for i in range(3):
#     for data in data_v2:
#         t = data[:, 0]
#         f = abs(data[:, i+1])
#         ax[i, 0].plot(t, f, '_')
#     for data in data_v3:
#         t = data[:, 0]
#         f = abs(data[:, i+1])
#         ax[i, 1].plot(t, f, '_')

# for a in ax.flatten():
#     # a.legend()
#     a.grid()

fig, ax = plt.subplots(1, 3, sharey=True, layout='constrained', figsize=(10, 5))

# ax[0].set_aspect('equal', 'box')
# ax[1].set_aspect('equal', 'box')

metrics_v2 = []
metrics_v3 = []
metrics_v4 = []

for data in data_v2:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    metrics_v2.append(m)

    ax[0].plot(t, f, '_')

for data in data_v3:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    metrics_v3.append(m)

    ax[1].plot(t, f, '_')

for data in data_v4:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    metrics_v4.append(m)

    ax[2].plot(t, f, '_')

ax[0].set_ylabel('$\|\|f_{ext}\|\|$', fontsize=20)

ax[0].set_xlabel('Time (s)', fontsize=20)
ax[1].set_xlabel('Time (s)', fontsize=20)
ax[2].set_xlabel('Time (s)', fontsize=20)

ax[0].set_title('Without FF', fontsize=20)
ax[1].set_title('With FF', fontsize=20)
ax[2].set_title('With FF (reform)', fontsize=20)

for a in ax.flatten():
    a.grid()

for tick in ax[0].xaxis.get_major_ticks():
    tick.label.set_fontsize(20)

for tick in ax[0].yaxis.get_major_ticks():
    tick.label.set_fontsize(20)

for tick in ax[1].xaxis.get_major_ticks():
    tick.label.set_fontsize(20)

for tick in ax[1].yaxis.get_major_ticks():
    tick.label.set_fontsize(20)

for tick in ax[2].xaxis.get_major_ticks():
    tick.label.set_fontsize(20)

for tick in ax[2].yaxis.get_major_ticks():
    tick.label.set_fontsize(20)

fig, ax = plt.subplots(layout='constrained')

mm2 = np.mean(metrics_v2)
mm3 = np.mean(metrics_v3)
mm4 = np.mean(metrics_v4)

sd2 = np.std(metrics_v2)
sd3 = np.std(metrics_v3)
sd4 = np.std(metrics_v4)

versions = ['Without FF', 'With FF', 'With FF (reform)']
x_pos = np.arange(len(versions))
means = [mm2, mm3, mm4]
error = [sd2, sd3, sd4]

ax.bar(x_pos, means, yerr=error, align='center', ecolor='black', capsize=10)
ax.set_xticks(x_pos)
ax.set_xticklabels(versions, fontsize=20)
ax.yaxis.grid(True)
ax.set_ylabel('Performance Metric', fontsize=20)

for tick in ax.yaxis.get_major_ticks():
    tick.label.set_fontsize(20)

print(f"{metrics_v2=}")
print(f"{metrics_v3=}")
print(f"{metrics_v4=}")

print("Mean (v2/3/4):", mm2, mm3, mm4)
print("Std (v2/3/4):", sd2, sd3, sd4)

plt.show()
