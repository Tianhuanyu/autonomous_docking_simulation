import os

def filenames(startwith):
    for filename in os.listdir('data/plot_f_ext_updated'):
        if filename.startswith(startwith):
            yield filename

def num_files(startwith):
    return len([f for f in filenames(startwith)])

total_v2 = num_files('f_ext_v2')
total_v3 = num_files('f_ext_v3')
total_v4 = num_files('f_ext_v4')
total_nn = num_files('f_ext_nn') - num_files('f_ext_nn_2') - num_files('f_ext_nn_hybrid')
total_nn_2 = num_files('f_ext_nn_2')
total_nn_hybrid = num_files('f_ext_nn_hybrid')

num_success_v2 = num_files('f_ext_v2_success')
num_success_v3 = num_files('f_ext_v3_success')
num_success_v4 = num_files('f_ext_v4_success')
num_success_nn = num_files('f_ext_nn_success')
num_success_nn_2 = num_files('f_ext_nn_2_success')
num_success_nn_hybrid = num_files('f_ext_nn_hybrid_success')

def score(num_success, total):
    return 100.*float(num_success)/float(total)

print("Total")
print("  V2:", total_v2)
print("  V3:", total_v3)
print("  V4:", total_v4)
print("  Vnn:", total_nn)
print("  Vnn_2:", total_nn_2)
print("  Vnn_hybrid:", total_nn_hybrid)

print("Success rate")
print("  V2:", score(num_success_v2, total_v2))
print("  V3:", score(num_success_v3, total_v3))
print("  V4:", score(num_success_v4, total_v4))
print("  Vnn:", score(num_success_nn, total_nn))
print("  Vnn_2:", score(num_success_nn_2, total_nn_2))
print("  Vnn_hybrid:", score(num_success_nn_hybrid, total_nn_hybrid))
