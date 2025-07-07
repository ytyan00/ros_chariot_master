#!/python3
# joystick based on: https://www.kernel.org/doc/Documentation/input/joystick-api.txt

#Requires: socketCan, can0 interface

# This file is part of can2RNET.
#
# can2RNET is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# can2RNET is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Foobar.  If not, see <http://www.gnu.org/licenses/>.

import socket, sys, os, array, threading
from time import *
from fcntl import ioctl
from can2RNET import *

#         joyframe = '02000200'+'#'+dec2hex(0,2)+dec2hex(180,2)
#         cansend(can_socket, joyframe)

can_socket = opencansocket(0)

# rnet_joystick_id = RNET_JSMerror_exploit(can_socket)

#########################################################################




def close():
    print('close')
    joyframe = '062#40010000' 
    cansend(can_socket, joyframe)

    ### see comments in open(). Again, not sure why we drop these two signals
    # joyframe = '061#52010002' 
    # cansend(can_socket, joyframe)

    # joyframe = '061#60010000' 
    # cansend(can_socket, joyframe)

    joyframe = '061#80010010' 
    cansend(can_socket, joyframe)


    joyframe = '062#00000000' 
    cansend(can_socket, joyframe)


    joyframe = '061#12000002' 
    cansend(can_socket, joyframe)

    joyframe = '061#20000000' 
    cansend(can_socket, joyframe)

    joyframe = '061#30000001' 
    cansend(can_socket, joyframe)

    joyframe = '061#80000080' 
    cansend(can_socket, joyframe)



    # change ui
    # joyframe = '0C180100#0101' 
    # cansend(can_socket, joyframe)  

    # # set speed
    # joyframe = '0A040200#64' 
    # cansend(can_socket, joyframe)   

    # joyframe = '181C0200#0260000000000000' 
    # cansend(can_socket, joyframe)    


    joyframe = '02000200#0000'
    cansend(can_socket, joyframe)

    joyframe = '02000200#0000'
    cansend(can_socket, joyframe)
    joyframe = '02000200#0000'
    cansend(can_socket, joyframe)


    
    
    
def next():
    for i in range(30000):
        joyframe = '02000200#0000'
        cansend(can_socket, joyframe)
    # next one
    for i in range(3000):
        joyframe = '02000200#6204'
        cansend(can_socket, joyframe)

    for i in range(30000):
        joyframe = '02000200#0000'
        cansend(can_socket, joyframe)


open()
# next()


    
# # next one
# for i in range(30000):
#     joyframe = '02000200#6204'
#     cansend(can_socket, joyframe)






close()



for i in range(30000):
    joyframe = '02000200#10A0'
    cansend(can_socket, joyframe)


