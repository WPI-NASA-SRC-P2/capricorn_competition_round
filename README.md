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

### rviz
To start the rviz, execute `src_sim_rviz`

### Terminal
To attach a terminal to the docker, execute `src_sim_terminal`


# Developing and Testing
#### Download the repository
Create a workspace and clone the repository
```bash
mkdir -p ~/catkin_ws/src 
cd catkin_ws/src/ && git clone https://github.com/WPI-NASA-SRC-P2/capricorn_competition_round.git
cd ~/catkin_ws/ && catkin_make
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

>For executing any command within the development container, use the VScode terminal. This terminal will be running within the development container. You can open this terminal by pressing: `ctrl +`\` (ctrl + tilde key)

For getting the command line access to the docker, execute `src_comp_terminal`
