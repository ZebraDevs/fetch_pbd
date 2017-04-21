#!/bin/bash
./node_modules/polymer-cli/bin/polymer.js build
rm -r compiled_frontend
mv build compiled_frontend
