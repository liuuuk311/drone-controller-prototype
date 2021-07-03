#!/bin/bash

rsync -p -r --exclude 'venv' '.vscode' * pi@raspberrypi:/home/pi/code