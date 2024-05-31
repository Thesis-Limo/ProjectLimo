# How to use your pc as monitor for the Limo

Make sure the limo is turned on
Now this has been gone easier. in each terminal you can run

```bash
limo_ip "ipaddresslimo"
```

for example

```bash
limo_ip 10.22.138.131
```

# How to run pipeline in Limo

After sync with the limo, with the github,
Run the file dependencyLimo, here you first need to run the dependency if changed, then run the build. So then the build is finished and start running the pipeline

# How to run without sim and rviz

You can run the environment in docker and use rviz and the simulator on your pc that provide the information to docker. Run the file

```bash
./run
```

select NoSim, and then roscore and terminal will be started. For easy use build inside this container, it has the dependencies needed to build it.

Then gazebo and rviz can be started on the pc without using docker.

# Set up project

After running the terminal with _./run_, execute the following commands:

```bash
pip3 install -r requirements.txt
```

To install all the requirements

```bash
build_planner
```

To build the motion planner

```bash
cbuild
```

To build the rest

# run limo

```bash
roslaunch limo_launch limo_pipeline.launch
```

This will run the full pipeline

# github ssh certification on limo

2868
