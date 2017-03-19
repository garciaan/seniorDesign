#!/bin/bash
if [ `uname` == 'Darwin' ]
then
	echo "MacOS Detected"
	sudo easy_install pip
	sudo pip install pyserial

elif [ `uname` == 'Linux' ]
then
	echo "Linux Detected"
	sudo apt-get update -y
	sudo apt-get upgrade -y
	sudo apt-get install python3-serial -y
	sudo apt-get install python3-pip -y
	sudo pip3 install --upgrade pip
	sudo pip3 install pygame
	sudo pip3 install pyqt5
else
	echo "Unkown Operating System"
fi
