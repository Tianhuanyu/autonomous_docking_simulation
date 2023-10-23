import os
import sys
import shutil
import subprocess

def run_expr(name):
    subprocess.run(['python', f'plan_peg_in_hole_unfixed_{name}.py', 'save'])

num = int(sys.argv[1])
for _ in range(num):
    run_expr('v2')
    run_expr('v3')
    run_expr('v4')
    run_expr('nn')
    run_expr('nn_2')
    run_expr('nn_hybrid')

for filename in os.listdir('data'):
    if filename.endswith('.csv'):
        src = 'data/' + filename
        dst = 'data/plot_f_ext_updated/' + filename
        shutil.move(src, dst)

subprocess.run(['python', 'describe_expr_success.py'])
