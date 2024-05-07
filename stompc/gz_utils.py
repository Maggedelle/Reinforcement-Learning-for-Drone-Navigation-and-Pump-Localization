import os
import sys
import signal
from subprocess import Popen, PIPE

xrce_process = None
gz_process = None
launch_process = None
xrce_cmd = 'MicroXRCEAgent udp4 -p 8888'
gz_cmd = 'PX4_SYS_AUTOSTART=4002 HEADLESS=1 PX4_GZ_MODEL_POSE="-4.0,2.0,0.24" make px4_sitl gz_x500_depth'
launch_file = 'bridges_and_nodes_launch.py'

# SHOULD NOT BE USED, KEPT IN BECAUSE IT MIGHT BE FIXED LATER!!
def run_launch_file(LAUNCH_PATH: str):
    global launch_process
    print('Launching slam toolbox, PointCloud2Laserscan and all bridges')
    launch_process = Popen('ros2 launch {}/{}'.format(LAUNCH_PATH,launch_file),
                           shell=True,
                           stdout=PIPE,
                           stderr=PIPE,
                           )
    print("launch file pid:",launch_process.pid)
    
def kill_launch_file():
    print('Killing launch file')
    os.kill(launch_process.pid, signal.SIGTERM)
    os.kill(launch_process.pid+2, signal.SIGTERM)
    os.kill(launch_process.pid+4, signal.SIGTERM)
    os.kill(launch_process.pid+6, signal.SIGTERM)
    os.kill(launch_process.pid+8, signal.SIGTERM)
    #launch_process.terminate()


def run_gz(GZ_PATH: str):
    global gz_process
    print('starting gz')
    gz_process = Popen('cd {} && {}'.format(GZ_PATH, gz_cmd),
                       shell=True,
                       stdout=PIPE,
                       stderr=PIPE,
                       )
    print('gazebo pid:', gz_process.pid)

def kill_gz():
    global gz_process
    print('Killing gz')
    os.kill(gz_process.pid+1)

def run_xrce_agent():
    global xrce_process
    print('Starting micro agent')
    xrce_process = Popen(xrce_cmd,
                        shell=True,
                        stdout=PIPE,
                        stderr=PIPE,
                        )
    print('xrce pid:', xrce_process.pid)
    
def kill_xrce_agent():
    global xrce_process
    print('Killing micro agent')
    os.kill(xrce_process.pid+1, signal.SIGINT)
    #xrce_process.terminate()