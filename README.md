# Contributing to the project

* Clone the repository
* Create a new local branch (ex: sg/basic-controls)
* Make changes to your branch with impunity
* Push local branch to git (and commit and push updates frequently)
* Whenever you're ready (ex: you've finished a feature), make a pull request (this will merge your changes into the main branch)
* (Software team lead will be responsible for timely review of pull requests)

# To Run

Load code from "sensors" environment onto sensors microcontroller
Load code from "robot" enviroment onto main microcontroller
Start both sets of code (by hitting reset on both microcontrollers)

Connect your laptop to the jetson via the usb c port. Run:
ssh lilo@192.168.55.1
Your terminal is now connected to the jetson terminal. You should be able to:
cd Downloads/2.12/mobilerobot
pyenv activate localization
sudo chmod 777 /dev/ttyACM0 
sudo chmod 777 /dev/ttyACM1
export PYTHONPATH=.
python ./vision/friday.py
(password is admin, btw)

NOTE: must run from mobilerobot directory (and that's the directory that PYTHONPATH should point to as well)

# Test

# Project structure

# Swati Random Notes

Start pyenv automatically
Give port permission: sudo chmod 777 /dev/ttyACM0
aka port itself dymanic (0, 1, 2)
order is open serial monitor on esp, it asks for a specific port, then before giving it permission to that port have to open that port on the jetson 
