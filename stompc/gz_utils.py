import os
import signal
from subprocess import Popen, PIPE

xrce_process = None
gz_process = None
launch_process = None
xrce_cmd = 'MicroXRCEAgent udp4 -p 8888'
gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=0 PX4_GZ_MODEL_POSE="-4.0,2.0,0.24" make px4_sitl gz_x500_depth'
launch_file = 'bridges_and_nodes_launch.py'

# SHOULD NOT BE USED, KEPT IN BECAUSE IT MIGHT BE FIXED LATER!!
def run_launch_file(LAUNCH_PATH: str):
    global launch_process
    print('Launching slam toolbox, PointCloud2Laserscan and all bridges')
    launch_process = Popen('ros2 launch {}/{}'.format(LAUNCH_PATH,launch_file),
                           shell=True
                           #stdout=PIPE,
                           )
    
def kill_launch_file():
    print('Killing launch file')
    launch_process.send_signal(signal.SIGINT)

def run_gz(GZ_PATH: str):
    global gz_process
    print('starting gz')
    gz_process = Popen('cd {} && {}'.format(GZ_PATH, gz_cmd),
                       shell=True,
                       stdout=PIPE,
                       stderr=PIPE,
                       )

def kill_gz():
    global gz_process
    print('Killing gz')
    gz_process.kill()

def run_xrce_agent():
    global xrce_process
    print('Starting micro agent')
    xrce_process = Popen(xrce_cmd,
                        shell=True,
                        stdout=PIPE,
                        stderr=PIPE,
                        )
    
def kill_xrce_agent():
    global xrce_process
    print('Killing micro agent')
    xrce_process.kill()