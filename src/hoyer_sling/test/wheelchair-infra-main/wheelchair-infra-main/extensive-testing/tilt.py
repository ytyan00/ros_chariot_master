import socket, sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *

cansocket = opencansocket(0)
joyframe = '1C2C0200#00300B860717'
while True:
  cansend(cansocket, joyframe)
