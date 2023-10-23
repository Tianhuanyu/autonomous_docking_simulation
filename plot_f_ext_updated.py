import os
import random
import numpy as np
import matplotlib.pyplot as plt

tick_fs = 15

def filenames(startwith):
    for filename in os.listdir('data/plot_f_ext_updated'):
        if filename.startswith(startwith):
            yield filename

def load_datasets(startwith):
    return [
        np.loadtxt('data/plot_f_ext_updated/'+filename, delimiter=',')
        for filename in filenames(startwith)
    ]

data_v2 = load_datasets('f_ext_v2_success')
data_v3 = load_datasets('f_ext_v3_success')
data_v4 = load_datasets('f_ext_v4_success')
data_nn = load_datasets('f_ext_nn_success')
data_nn_2 = load_datasets('f_ext_nn_2_success')
data_nn_hybrid = load_datasets('f_ext_nn_hybrid_success')

random.shuffle(data_v2)
random.shuffle(data_v3)
random.shuffle(data_v4)
random.shuffle(data_nn)
random.shuffle(data_nn_2)
random.shuffle(data_nn_hybrid)

max_num_success = min(len(data_v2), len(data_v3), len(data_v4), len(data_nn), len(data_nn_2), len(data_nn_hybrid))
print(len(data_v2), len(data_v3), len(data_v4), len(data_nn), len(data_nn_2), len(data_nn_hybrid))
print(f"{max_num_success=}")

data_v2 = data_v2[:max_num_success]
data_v3 = data_v3[:max_num_success]
data_v4 = data_v4[:max_num_success]
data_nn = data_nn[:max_num_success]
data_nn_2 = data_nn_2[:max_num_success]
data_nn_hybrid = data_nn_hybrid[:max_num_success]

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

fig, ax = plt.subplots(1, 6, sharey=True, layout='constrained', figsize=(10, 5))

# ax[0].set_aspect('equal', 'box')
# ax[1].set_aspect('equal', 'box')

metrics_v2 = []
metrics_v3 = []
metrics_v4 = []
metrics_nn = []
metrics_nn_2 = []
metrics_nn_hybrid = []

max_t_v2 = -1
max_t_v3 = -1
max_t_v4 = -1
max_t_nn = -1
max_t_nn_2 = -1
max_t_nn_hybrid = -1

for data in data_v2:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_v2:
        max_t_v2 = t[-1]

    metrics_v2.append(m)

    ax[0].plot(t, f, '_')

for data in data_v3:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_v3:
        max_t_v3 = t[-1]

    metrics_v3.append(m)

    ax[1].plot(t, f, '_')

for data in data_v4:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_v4:
        max_t_v4 = t[-1]

    metrics_v4.append(m)

    ax[2].plot(t, f, '_')

for data in data_nn:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_nn:
        max_t_nn = t[-1]

    metrics_nn.append(m)

    ax[3].plot(t, f, '_')

for data in data_nn_2:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_nn_2:
        max_t_nn_2 = t[-1]

    metrics_nn_2.append(m)

    ax[4].plot(t, f, '_')

for data in data_nn_hybrid:
    t = data[:, 0]
    f = np.linalg.norm(data[:, 1:4], axis=1)

    m = 0.
    for i in range(len(t)):
        m += f[i]
    m/=t[-1]

    if t[-1] > max_t_nn_hybrid:
        max_t_nn_hybrid = t[-1]

    metrics_nn_hybrid.append(m)

    ax[5].plot(t, f, '_')

def mean_duration(datas):
    return np.mean([data[-1, 0] for data in datas])

def std_duration(datas):
    return np.std([data[-1, 0] for data in datas])

def max_duration(datas):
    return max([data[-1, 0] for data in datas])

print("Time")
print("  v2", mean_duration(data_v2), std_duration(data_v2), max_duration(data_v2))
print("  v3", mean_duration(data_v3), std_duration(data_v3), max_duration(data_v3))
print("  v4", mean_duration(data_v4), std_duration(data_v4), max_duration(data_v4))
print("  nn", mean_duration(data_nn), std_duration(data_nn), max_duration(data_nn))
print("  nn_2", mean_duration(data_nn_2), std_duration(data_nn_2), max_duration(data_nn_2))
print("  nn_hybrid", mean_duration(data_nn_hybrid), std_duration(data_nn_hybrid), max_duration(data_nn_hybrid))

ax[0].set_ylabel('$\|\|f_{ext}\|\|$', fontsize=20)

ax[0].set_xlabel('Time (s)', fontsize=20)
ax[1].set_xlabel('Time (s)', fontsize=20)
ax[2].set_xlabel('Time (s)', fontsize=20)
ax[3].set_xlabel('Time (s)', fontsize=20)
ax[4].set_xlabel('Time (s)', fontsize=20)
ax[5].set_xlabel('Time (s)', fontsize=20)

ax[0].set_title('Without FF (50%)', fontsize=20)
ax[1].set_title('With FF (47%)', fontsize=20)
ax[2].set_title('With FF [reform] (94%)', fontsize=20)
ax[3].set_title('BC (20%)', fontsize=20)
ax[4].set_title('BC with FF (30%)', fontsize=20)
ax[5].set_title('BC hybrid (34%)', fontsize=20)

for a in ax.flatten():
    a.grid()

for tick in ax[0].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[0].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[1].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[1].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[2].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[2].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[3].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[3].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[4].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[4].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[5].xaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

for tick in ax[5].yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

fig, ax = plt.subplots(layout='constrained')

mm2 = np.mean(metrics_v2)
mm3 = np.mean(metrics_v3)
mm4 = np.mean(metrics_v4)
mmnn = np.mean(metrics_nn)
mmnn_2 = np.mean(metrics_nn_2)
mmnn_hybrid = np.mean(metrics_nn_hybrid)

sd2 = np.std(metrics_v2)
sd3 = np.std(metrics_v3)
sd4 = np.std(metrics_v4)
sdnn = np.std(metrics_nn)
sdnn_2 = np.std(metrics_nn_2)
sdnn_hyrbid = np.std(metrics_nn_hybrid)

# Success rate
#   V2: 50.0
#   V3: 46.666666666666664
#   V4: 94.0
#   Vnn: 20.0
#   Vnn_2: 30.0
#   Vnn_hybrid: 34.0  


versions = ['Without FF', 'With FF', 'With FF [reform]', 'BC', 'BC with FF', 'BC hybrid']
x_pos = np.arange(len(versions))
means = [mm2, mm3, mm4, mmnn, mmnn_2, mmnn_hybrid]
error = [sd2, sd3, sd4, sdnn, sdnn_2, sdnn_hyrbid]

ax.bar(x_pos, means, yerr=error, align='center', ecolor='black', capsize=10)
ax.set_xticks(x_pos)
ax.set_xticklabels(versions, fontsize=20)
ax.yaxis.grid(True)
ax.set_ylabel('Performance Metric', fontsize=20)

for tick in ax.yaxis.get_major_ticks():
    tick.label.set_fontsize(tick_fs)

# print(f"{metrics_v2=}")
# print(f"{metrics_v3=}")
# print(f"{metrics_v4=}")
# print(f"{metrics_nn=}")
# print(f"{metrics_nn_2=}")
# print(f"{metrics_nn_hybrid=}")

print("Mean (v2/3/4/nn/nn_2/nn_hyrbid):", mm2, mm3, mm4, mmnn, mmnn_2, mmnn_hybrid)
print("Std (v2/3/4/nn/nn_2/nn_hyrbid):", sd2, sd3, sd4, sdnn, sdnn_2, sdnn_hyrbid)

plt.show()
