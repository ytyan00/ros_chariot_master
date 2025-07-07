# wheelchair-infra

# The repo provides interfaces to control R-net electric wheelchair system.
1. please read through the whole readme before you start setting up the system

## Set up raspberry pi and wheelchair 
1. To set up rp and wheelchair, please follow [wheelchair integration tutorial](https://docs.google.com/document/d/1ea7M7u7Ypgo3TlnYB3_JpQVOWl27jFGPHbgxNdkJFbY/edit) 
2. To set up realsense, please follow [realsense-ros tutorial](https://docs.google.com/document/d/161s89tRV-lP5Wkoh4dgJHh2bsaMx1oewh1cC9f0lFwk/edit)
3. please ssh into rp and clone this repo to rp

## controlling examples


### keyboard control

1. please go to folder 'keyboard_control' and run "python keyboard_control.py"

### xbox joystick control

1. please make sure you have connected an xbox joystick to rp.
2. go to folder "xbox_control/can2RNET" and run "python JoyLocal.py"


### Camera Control
1. please check the above tutorials and set up your realsense cameras

