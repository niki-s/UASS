#!bin/bash

#Welcome to RosAria!
#This script should get everything running for you nice and easy
#copy outside of folder to catkin/devel before running
echo "Welcome to RosAria"
echo "Reading config..."

source UASS/UASS.cfg

echo "***launching Roscore services"
gnome-terminal -x bash -c './UASS/roscoreDriver.sh; bash' &

#wait
sleep 5

echo
echo "***launching RosAria"
gnome-terminal -x bash -c './UASS/rosAriaDriver.sh; bash' &

#wait
sleep 5

echo
echo "***launching position_listener"
gnome-terminal -x bash -c './UASS/positionDriver.sh; bash' &

echo
echo "***launching controller"
gnome-terminal -x bash -c './../src/rosaria/controller.py; bash' &

echo
echo "package is now running"

wait
