#!/bin/bash
echo $(pwd)
PACKAGE_DIR=$(ros2 pkg prefix route_admin_panel)
cd $PACKAGE_DIR/share/route_admin_panel/nodejs
node main.js $@