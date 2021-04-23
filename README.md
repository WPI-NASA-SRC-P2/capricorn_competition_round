# [Important Links Sheet](https://docs.google.com/spreadsheets/d/1_u9gZq9JMEhlwfY9OdAV7LRMA7wjt9MWbYRa9ZyF7W4/edit?usp=sharing)
Google sheet link for all the links to important stuff. Feel free to add new links if you find any.


# Working with the simulation
## Setup Simulation
#### Downloading the simulation:
Clone the repository if you haven't done it already. _(Enter your GitHub credentials when prompted)_
```bash
cd && git clone https://github.com/WPI-NASA-SRC-P2/srcp2-final-public.git
```
Please read [the wiki](https://github.com/WPI-NASA-SRC-P2/srcp2-final-public/wiki/2.-Requirements,-Install-and-Updates)  to install the sim.


#### Setting the aliases for easing the usage:
Add the sourcing line to the `bashrc` by executing this command:
```bash
echo "source ~/srcp2-final-public/aliases.bash" >> ~/.bashrc
````

## Starting and stopping the Simulation
With the aliases set above, simulation should start with a command `src_start_simulation`, and can be killed with a command `src_stop_simulation`

_(For the first time, you might have to execute `source ~/.bashrc` or use a new terminal)_

### Terminal
To attach a terminal to the simulation docker, execute `src_sim_terminal`

_(You won't usually need this access, but the alias has been set nonetheless)_


# Developing and Testing
#### Download the repository
Create a workspace and clone the repository
```bash
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/src/ && git clone https://github.com/WPI-NASA-SRC-P2/capricorn_competition_round.git
```

#### Setup Visual Studio Code
Install and setup Visual Studio Code for developing with CPP with the instructions [given here](https://github.com/WPI-NASA-SRC-P2/TeamCapricorn/wiki/Visual-Studio-Code-for-ROS-with-CPP)

Install the following extensions for VSCode
1. Docker
2. Remote-containers

## Developing with VSC
1. Start the development container 
```bash
src_start_comp_docker
```

Click on Docker extension, right click on the running container from previous step and click Attach Visual Studio Code
![](https://github.com/WPI-NASA-SRC-P2/capricorn_docker/blob/main/vsc_setup.gif)

>For executing any command within the development container, use the VScode terminal. This terminal will be running within the development container. You can open this terminal by pressing: `ctrl +`\` (ctrl + t
>ilde key)

For getting the command line access to the docker, execute `src_comp_terminal`


## System Setup

### System Setup Instruction

1. Update your srcp2-final-public and capricorn-competition-round repository
1.A. To update "srcp2-final-public" repository and run commands:
```bash
cd srcp2-final-public
git pull
```
1.B. Similar for the capricorn-competition-round repository and run commands:
```bash
cd ~/catkin_ws/src/capricorn-competiton-round
git pull
```
2. To start the simulation, open the terminal and run the command:
```bash
src_start_simulation
```
2.A. If you get an error, "command not found" or "container not found", there is an issue with sourcing the alias. To fix it, you can run the command:
```bash
echo "source ~/srcp2-final-public/aliases.bash" >> ~/.bashrc
```
2.B. Restart the terminal
3. To develop the code you have to start the comp_docker. To do this open a new terminal, run comand:
```bash
src_start_comp_docker
```
### RUN ALL THE COMMANDS AFTER THIS POINT IN THE comp_docker TERMINAL

4. Remove all other packages from ~/catkin_ws/src and only keep "capricorn-competition-round"
5. To build the capricorn-competition-round repository, run commands:
```bash
cd ~/catkin_ws
catkin build
```

5.A. if this doesn't work, delete devel and build folder in catkin_ws and run commands:
```bash
catkin init
catkin clean
catkin build
```
6. You would need to source all the packages and to do this run command:
```bash
source ~/catkin_ws/devel/setup.bash
```
7. To test if you can run code, execute the command:
```bash
roslaunch maploc rtabmap.launch
```
















