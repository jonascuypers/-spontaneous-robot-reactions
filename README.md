# Master Thesis: spontaneous robot reactions 
## Introduction
This repository contains the source code accompanying my master thesis. 
The software was created to cooperate with a Pepper robot, but this was not tested due to COVID-19. The software does work 
with the NAO bot in simulation. 

The master thesis uses of [ROS kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu) and the code is written in [Python 2.7](https://www.python.org/download/releases/2.7/).

## Installation
### Prerequisites
- [Install Python 2.7](https://www.python.org/download/releases/2.7/)
- [Install ROS kinetic](https://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Install the Python Naoqi SDK](http://doc.aldebaran.com/2-5/dev/python/install_guide.html)
- [Install the Webots 8 simulator for Nao](http://doc.aldebaran.com/2-1/software/webots/webots_index.html)
### Install
- Clone this repository
- Install *requirements.txt* using pip
- Clone [ROSPlan](https://github.com/KCL-Planning/ROSPlan) as a submodule 
- Install [ROSPlan](https://github.com/KCL-Planning/ROSPlan)
- Install [Polly TTS](https://github.com/aws-robotics/tts-ros1)
- Make sure the parameter *recoring_path* is set to a existing directory

## Usage
Start the Webots 8 simulator. The file with the appartment and NAO is located here:
 *src/initialization/appartment_NOA.wbt*

For running all nodes run *roslaunch src/initialisation/launch_all.launch*