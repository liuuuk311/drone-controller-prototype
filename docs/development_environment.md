# Development Environment

In order to build your local development environment, you'll need 
[docker](https://www.docker.com/products/docker-desktop) installed on your machine.

We need Docker to run the simulation environment. To run it go to the terminal and just type:
```bash
docker run --rm -it jonasvautherin/px4-gazebo-headless:1.11.0
```

> You can also add some env variables to tell the simulator where to place the drone. 
> 
> For example: `--env PX4_HOME_LAT=<lat> --env PX4_HOME_LON=<lon>`

By doing this, we have our simulated flight controller up and running, ready to receive 
[MAVLink](https://mavlink.io/en/) commands.

## Drone Controller

To install the drone controller we need to install its dependencies first. Let's create a python virtual environment to 
keep things tidy. Open a terminal window inside the project folder and type:
```bash
python3 -m venv /venv
source /venv/bin/activate
pip3 install -r requirements.txt
```

Now once we installed everything, we can run our code by simply type:
```bash
python3 main.py
```

## Structure of the project
The following files and folders are the actual project. We are going to explain briefly the meaning of each one of them.
```
├── aiofsm
├── core
├── data
├── docs
├── main.py
└── tests
```

### aiofsm
A lightweight, decorator-based Python implementation of a Finite State Machine that supports asyncIO.

### core
The core of the project, here there are all the files responsible to control the drone.

### data
In this folder will be stored all the mission plans data.

### docs
This folder contains all the documentation of this project

### main.py
The entrypoint of the project. Run this script to run the project

### tests
All the unit tests files are in this folder
 