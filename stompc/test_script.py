import os
import signal
from subprocess import Popen, PIPE
import time
import psutil
import csv

NUMBER_OF_RUNS = 2
MAX_TIME_PER_RUN = 600

def kill_proc_tree(pid, sig=signal.SIGINT, include_parent=True,
                   timeout=None, on_terminate=None):
    """Kill a process tree (including grandchildren) with signal
    "sig" and return a (gone, still_alive) tuple.
    "on_terminate", if specified, is a callback function which is
    called as soon as a child terminates.
    """
    assert pid != os.getpid(), "won't kill myself"
    parent = psutil.Process(pid)
    children = parent.children(recursive=True)
    if include_parent:
        children.append(parent)
    for p in children:
        try:
            p.send_signal(sig)
        except psutil.NoSuchProcess:
            pass
    gone, alive = psutil.wait_procs(children, timeout=timeout,
                                    callback=on_terminate)
    
    gz_p = None
    try:
        for p in psutil.process_iter(['cmdline']):
            if p.info['cmdline'] and 'gz' in ' '.join(p.info['cmdline']):
                gz_p = p
        gz_p.kill()
    except (psutil.AccessDenied, psutil.NoSuchProcess):
        print('no process with gz :thinking:')
        pass


    return (gone, alive)

file = 'Experiment_open=1_turningcost=20_movingcost=20_discoveryreward=10_pumpreward=1000_safetyrange=40cm_maxiter=6_rnb=3_gr=300_tr=300_rps=100.csv'

def get_number_of_lines_csv (filename):
    number_of_lines = 0
    with open(filename) as testfile:
        reader = csv.reader(testfile, delimiter=',')
        number_of_lines = len([row for row in reader])
    return number_of_lines - 1

i = 0
number_of_lines = 0
while number_of_lines < NUMBER_OF_RUNS:
    print("starting run {}".format(i+1))
    start_time = time.time()
    stompc_proc = psutil.Popen("python3 stompc.py",
                    shell=True,
                    stderr=PIPE,
                    )
    curr_time = 0
    while curr_time + 20 < MAX_TIME_PER_RUN:
        time.sleep(30)
        check_len = get_number_of_lines_csv(file)
        print(check_len, number_of_lines)
        curr_time = time.time() - start_time
        print("Time spent so far for run {}: {} seconds".format(i+1, curr_time))
        if check_len > number_of_lines:
            number_of_lines = check_len
            print('Run seems to be finished. Proceeding to kill processes')
            break

    print("Run {} finished, killing processes".format(i+1))
    kill_proc_tree(stompc_proc.pid)
    #os.kill(stompc_proc.pid, signal.SIGINT)
    #cleanup_gz()
    time.sleep(5)
    print("Processes killed")