# Master Thesis: spontaneous robot reactions 
## Introduction
This repository contains the source code accompanying my master theses. 
The software was created to cooperate with a Pepper robot, but this was not tested. 

The master thesis uses of [ROS kinetic]() and the code is written in [Python 2.7]().

## Installation
### Prerequisites
- [Install Python 2.7]()
- [Install ROS kinetic]()
- [Install the Python Naoqi SDK]()
- [Install the Webots 8 simulator for Nao]()
### Install
- Clone this repository
- Install *requirements.txt* using pip
- Clone [ROSPlan]() as a submodule 
- Install [Polly TTS]()
- Install [ROSPlan]()
- Make sure the parameter *recoring_path* is set to a existing directory

## Usage
Start the Webots 8 simulator with a Nao bot

For running all nodes run *src/initialisation/launch_all.launch*