#!/usr/bin/python3

import sys
import subprocess
import os
import argparse

parser = argparse.ArgumentParser(add_help=False)
parser.add_argument('-u', '--uninstall', dest='argument_uninstall', type=bool,
                    help="If uninstall", required=False, default=False)
parser.add_argument('-rc', '--roscore', dest='argument_roscore', type=bool,
                    help="Whether to install roscore autostart on host machine", default=False)
parser.add_argument('-hm', '--hostname', dest='argument_hostname', type=str,
                    help="Hostname", default="husarion")
parser.add_argument('-ri', '--rosip', dest='argument_rosip', type=str,
                    help="Local ip address", default="10.15.20.3")
parser.add_argument('-rmu', '--rosmasteruri', dest='argument_rosmasteruri', type=str,
                    help="Local ip address", default="10.15.20.2")
parser.add_argument('-p', '--panthertype', dest='argument_panthertype', type=str,
                    help="Local ip address", default="classic")
parser.add_argument('-h', '--help', action='help', default=argparse.SUPPRESS,
                    help='Show this help message and exit. Example usage: sudo python3 set_panther_upstart.py -rc False -hm husarion -ri 10.15.20.3 -rmu 10.15.20.2 -p classic')

args = parser.parse_args()
UNISTALL = args.argument_uninstall
ROSCORE = args.argument_roscore
HOSTNAME = args.argument_hostname
ROS_IP = args.argument_rosip
ROS_MASTER_URI = args.argument_rosmasteruri
PANTHER_TYPE = args.argument_panthertype


def prompt_sudo():
    ret = 0
    if os.geteuid() != 0:
        msg = "[sudo] password for %u:"
        ret = subprocess.check_call("sudo -v -p '%s'" % msg, shell=True)
    return ret


if prompt_sudo() != 0:
    sys.exit("The user wasn't authenticated as a sudoer, exiting")

if UNISTALL:
    subprocess.call("rm /usr/sbin/route_admin_panel.sh", shell=True)
    subprocess.call("rm /etc/ros/env.sh", shell=True)
    subprocess.call("rm /etc/systemd/system/roscore.service", shell=True)
    subprocess.call(
        "rm /etc/systemd/system/route_admin_panel.service", shell=True)
    subprocess.call("systemctl disable roscore.service", shell=True)
    subprocess.call("systemctl disable route_admin_panel.service", shell=True)
    sys.exit("All removed")


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

if ROSCORE:
    #
    # /etc/systemd/system/roscore.service
    #

    startup = "/bin/bash -c '. /opt/ros/noetic/setup.sh; . /etc/ros/env.sh; while ! ping -c 1 -n -w 1 10.15.20.1 &> /dev/null; do sleep 1; done ;roscore & while ! echo exit | nc {rmu} 11311 > /dev/null; do sleep 1; done'".format(
        rmu=ROS_MASTER_URI)

    roscore_service = """[Unit]
    After=NetworkManager.service time-sync.target
    [Service]
    TimeoutStartSec=60
    Restart=always
    RestartSec=0.5
    Type=forking
    User={hn}
    ExecStart={ex}
    [Install]
    WantedBy=multi-user.target
    """.format(hn=HOSTNAME, ex=startup)

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
until rostopic list
do
  sleep 1
done
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
After=NetworkManager.service time-sync.target
[Service]
Restart=always
RestartSec=10
Type=simple
User={hn}
ExecStart=/usr/sbin/route_admin_panel.sh
[Install]
WantedBy=multi-user.target
""".format(hn=HOSTNAME)

subprocess.Popen(
    ['echo "{}" > /etc/systemd/system/route_admin_panel.service'.format(rap_service)],  shell=True)

if ROSCORE:
    subprocess.call("systemctl enable roscore.service", shell=True)
subprocess.call("systemctl enable route_admin_panel.service", shell=True)
subprocess.call("chmod +x /usr/sbin/route_admin_panel.sh", shell=True)


print("Done!")
