import os
import subprocess

gz_process = None
gz_cmd = 'cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_MODEL_POSE="-4.0,2.0,0.24" make px4_sitl gz_x500_depth'
xrce_process = None
xrce_cmd = 'MicroXRCEAgent udp4 -p 8888'


def run_gz():
    global gz_process
    print('starting gz')
    gz_process = subprocess.Popen(gz_cmd,
                                  shell=True,
                                  )
    outs, errs = gz_process.communicate()

def kill_gz():
    global gz_process
    print('stopping gz')
    gz_process.kill()
    outs, errs = gz_process.communicate()

def run_xrce_agent():
    global xrce_process
    print('Starting micro agent')
    xrce_process = subprocess.Popen(xrce_cmd,
                                     shell=True,
                                     )
    outs, errs = xrce_process.communicate()
    
def kill_xrce_agent():
    global xrce_process
    print('Killing micro agent')
    xrce_process.kill()
    outs, errs = xrce_process.communicate()