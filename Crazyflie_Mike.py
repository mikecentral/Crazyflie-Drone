"""

This script shows some cool ways to control the Crazyflie 2.0 with FlowDeck.
THe code connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.
Much of the code comes from the motion_commander_demo.py example from Bitcraze

Change the URI variable to your Crazyflie configuration.

Thanks, Mike

"""

import logging
import time
import math
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework

logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':

    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)
            done = False

            while(not done):

                # cmd = input('Hi we are starting!') 
                
                # Spiral up and to the right then back down and to the left

                height = 1.0  # total spiral height in meters (don't go above sensor range)
                loops = 2.0   # number of loops to make
                vel = 0.7     # speed in m/s
                radius = 0.3  # radius in meters
                
                circumference = 2 * radius * math.pi
                loop_time = circumference/vel
                total_time = loops * loop_time

                vel_x = vel
                vel_y = vel
                vel_z = height/loops/loop_time
                yaw_rate = 360.0 / loop_time

                # up and to the right, facing the direction of motion (x motion)

                mc._set_vel_setpoint(vel_x, 0.0, vel_z, yaw_rate)
                time.sleep(total_time)
                mc.stop()

                time.sleep(2.0)

                # down and to the left, facing the inside of the spiral (y motion)
                
                mc.turn_right(90)
                mc._set_vel_setpoint(0.0, vel_y, -vel_z, -yaw_rate)
                time.sleep(total_time)
                mc.stop()

                done = True
     
            # Land the drone then close the connection
            
            print('Stopping')
            mc.stop()

            # We land when the MotionCommander goes out of scope