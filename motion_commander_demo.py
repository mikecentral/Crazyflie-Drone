"""

This script shows the basic use of the MotionCommander class.
Simple example that connects to the crazyflie at `URI` and runs a
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.

"""

import logging
import time

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

            # There is a set of functions that move a specific distance
            # We can move in all directions
            print ('Going Forward')
            mc.forward(0.8)
            mc.back(0.8)

            time.sleep(1)

            print ('Up and Down')
            mc.up(0.5)
            mc.down(0.5)

            time.sleep(1)

            # We can also set the velocity

            print ('Go right faster')
            mc.right(0.5, velocity=0.8)

            time.sleep(1)

            print ('Go left slower')
            mc.left(0.5, velocity=0.4)

            time.sleep(1)


            # We can do circles or parts of circles

            print('Make a circle')
            mc.circle_left(0.5, velocity=0.5, angle_degrees=360)

            # Or turn

            print ('turn to the left')
            mc.turn_left(90)
            time.sleep(1)

            # We can move along a line in 3D space

            print('Move along a line')
            mc.move_distance(-1, 0.0, 0.5, velocity=0.6)

            time.sleep(1)

            # There is also a set of functions that start a motion. The
            # Crazyflie will keep on going until it gets a new command.
            '''
            mc.start_left(velocity=0.5)

            # The motion is started and we can do other stuff, printing for
            # instance

            for _ in range(5):
                print('Doing other work')
                time.sleep(0.2)

            # And we can stop
            '''
            print('Stopping')
            mc.stop()

            # We land when the MotionCommander goes out of scope