#!/usr/bin/python3

import sys
import subprocess
import os

possible_arg_count = [2, 5]
if (len(sys.argv) not in possible_arg_count) or len(sys.argv) == 1:
    sys.exit("""
    ###Mismatch argument count. Expected 4.###

This script set up autostart of roscore and route_admin_panel 
USAGE: 
    sudo python3 set_panther_upstart.py <username> <local_ip_addr> <ros_master_ip/ros_master_hostname> <panther_type>
    
Example for default panther configuration: 
    sudo python3 set_panther_upstart.py husarion 10.15.20.3 10.15.20.3 classic

Uninstall: 
    sudo python3 set_panther_upstart.py uninstall
""")


def prompt_sudo():
    ret = 0
    if os.geteuid() != 0:
        msg = "[sudo] password for %u:"
        ret = subprocess.check_call("sudo -v -p '%s'" % msg, shell=True)
    return ret


if prompt_sudo() != 0:
    sys.exit("The user wasn't authenticated as a sudoer, exiting")

if str(sys.argv[1]) == "uninstall":
    subprocess.call("rm /usr/sbin/route_admin_panel.sh", shell=True)
    # subprocess.call("rm /usr/sbin/roscore_script.sh", shell=True)
    subprocess.call("rm /etc/ros/env.sh", shell=True)
    subprocess.call("rm /etc/systemd/system/roscore.service", shell=True)
    subprocess.call(
        "rm /etc/systemd/system/route_admin_panel.service", shell=True)
    subprocess.call("systemctl disable roscore.service", shell=True)
    subprocess.call("systemctl disable route_admin_panel.service", shell=True)
    sys.exit("All removed")


HOSTNAME = str(sys.argv[1])
ROS_IP = str(sys.argv[2])
ROS_MASTER_URI = str(sys.argv[3])
PANTHER_TYPE = str(sys.argv[4])

panther_types = ["mix", "classic", "mecanum"]
if PANTHER_TYPE not in panther_types:
    sys.exit("""
Wrong panther_type. Expected values:
    "mix", "classic", "mecanum"
    """)


print("Configuration ->", "Hostname:", HOSTNAME, "ROS_IP:", ROS_IP,
      "ROS_MASTER_URI:", ROS_MASTER_URI, "PANTHER_TYPE:", PANTHER_TYPE)
subprocess.call("mkdir /etc/ros", shell=True)


#
# /etc/ros/env.sh
#

env_msg = """#!/bin/sh
export ROS_IP={rip} 
export ROS_MASTER_URI=http://{rmu}:11311
""".format(rip=ROS_IP, rmu=ROS_MASTER_URI)

subprocess.Popen(['echo "{}" > /etc/ros/env.sh'.format(env_msg)],  shell=True)


#
# /etc/systemd/system/roscore.service
#

startup = "/bin/bash -c '. /opt/ros/noetic/setup.sh; . /etc/ros/env.sh; while ! ping -c 1 -n -w 1 {rmu} &> /dev/null;do  sleep 1;done;roscore & while ! echo exit | nc {rmu} 11311 > /dev/null; do sleep 1; done'".format(
    rmu=ROS_MASTER_URI)

roscore_service = """[Unit]
After=NetworkManager.service time-sync.target network-online.target
[Service]
Type=forking
User=husarion
ExecStart={}
[Install]
WantedBy=multi-user.target
""".format(startup)

subprocess.call("touch /etc/systemd/system/roscore.service", shell=True)
subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/roscore.service'.format(roscore_service)],  shell=True)


#
# /usr/sbin/route_admin_panel.sh
#

rap_script = """#!/bin/bash
source ~/husarion_ws/devel/setup.bash
source /etc/ros/env.sh
export ROS_HOME=$(echo ~{hn})/.ros
roslaunch route_admin_panel demo_panther_{pt}.launch &
PID=$!
wait "$PID"
""".format(hn=HOSTNAME, pt=PANTHER_TYPE)

subprocess.Popen(
    ['echo "{}" > /usr/sbin/route_admin_panel.sh'.format(rap_script)],  shell=True)


#
# route_admin_panel service
#

rap_service = """[Unit]
Requires=roscore.service
PartOf=roscore.service
After=NetworkManager.service time-sync.target roscore.service
[Service]
Type=simple
User={hn}
ExecStart=/usr/sbin/route_admin_panel.sh
[Install]
WantedBy=multi-user.target
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/route_admin_panel.service'.format(rap_service)],  shell=True)


subprocess.call("systemctl enable roscore.service", shell=True)
subprocess.call("systemctl enable route_admin_panel.service", shell=True)
subprocess.call("chmod +x /usr/sbin/route_admin_panel.sh", shell=True)
# subprocess.call("chmod +x /usr/sbin/roscore_script.sh", shell=True)


print("Done!")
