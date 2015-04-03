#!bin/bash

#Welcome to RosAria!
#This script should get everything running for you nice and easy
echo "Welcome to RosAria"
echo "Reading config..."

source UASS.cfg

echo "launching Roscore services"
gnome-terminal ./roscoreDriver.sh &

#wait
sleep 3

echo "launching RosAria"

#rosrun rosaira RosAria &

echo "launching position_listener"
