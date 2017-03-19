#!/bin/bash
if [ `uname` == 'Darwin' ]
then
	echo "MacOS Detected"
	sudo easy_install pip
	sudo pip install pyserial

elif [ `uname` == 'Linux' ]
then
	echo "Linux Detected"
	sudo apt-get update
	sudo apt-get upgrade
	sudo apt-get install python3-serial -y
	sudo apt-get install python3-pip -y
	sudo pip3 install --upgrade pip
	sudo pip3 install pygame
else
	echo "Unkown Operating System"
fi
