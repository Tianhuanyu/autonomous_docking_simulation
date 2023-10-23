import os
import sys
import shutil
import subprocess

def run_expr(ver_number=4):
    subprocess.run(['python', f'plan_peg_in_hole_unfixed_v{ver_number}.py', 'save'])

def filenames(startwith):
    for filename in os.listdir('data'):
        if filename.startswith(startwith):
            yield filename

def num_files(startwith):
    return len([f for f in filenames(startwith)])

num_demonstrations = int(sys.argv[1])

while num_files('f_ext_v4_success') < num_demonstrations:    
    run_expr()

for filename in filenames('f_ext_v4_failed'):
    os.remove('data/' + filename)

for filename in filenames('f_ext_v4_success'):
    src = 'data/' + filename
    dst = 'data/bc/' + filename
    shutil.move(src, dst)

print("Complete")
