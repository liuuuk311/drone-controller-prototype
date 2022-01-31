#!/bin/bash

rsync -p -r --exclude 'venv' --exclude '.vscode' --exclude 'tests' --exclude 'docs' --exclude 'data' --exclude 'README.md' --exclude 'requirements_dev.txt' --exclude 'remote_copy.sh' --exclude 'rasp_config_DS.sh' --exclude '.gitignore' * pi@raspberrypi.local:/home/pi/code/controller