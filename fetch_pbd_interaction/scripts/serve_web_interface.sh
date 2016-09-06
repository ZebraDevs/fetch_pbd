#!/bin/bash
path=`rospack find fetch_pbd_interaction`
cd $path/web_interface/fetch-pbd-gui
./node_modules/polymer-cli/bin/polymer.js serve --hostname 0.0.0.0
