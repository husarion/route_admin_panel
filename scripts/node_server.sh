#!/bin/bash
echo $(pwd)
PACKAGE_DIR=$(rospack find route_admin_panel)
cd $PACKAGE_DIR/nodejs
node main.js $@