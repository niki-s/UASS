Unmanned Autonomous System Swarm
================================

Code for Senior Projects 2014/2015 with the Univeristy of Nevada, Reno

rosaria
=======
The folder rosaria contains ros node code in C++ and python to 
control a Pioneer P3-DX with UASS. If using catkin, this folder
should be placed in the src directory and compiled with
catkin_make before running.

Ros and these control nodes are run using a bash script from UASS

UASS
====
This folder contains the bash scripts used to start UASS. If using
catkin, this folder should be placed in the devel directory and the
file marked start_uass_pioneer.sh should be placed outside of the 
folder.

To run:
$ roscd
$ bash start_uass_pioneer.sh
