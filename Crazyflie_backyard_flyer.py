# -*- coding: utf-8 -*-
"""
Intel Aero Version of the Backyard Flyer Project. On GitHub rev3
"""

import time
from enum import Enum
import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID

class States(Enum):

    # In Manual state the program has no control over the drone
    MANUAL = 0
    # In Arming state the program is taking control of the drone
    ARMING = 1
    # In Takeoff state the drone is climbing to the desired altitude at the origin
    TAKEOFF = 2
    # In Waypoint state the drone is flying the desired pattern
    WAYPOINT = 3
    # In Landing state the drone has reached its final x,y position and is landing
    LANDING = 4
    # In Disarming state the program turns the drone off and gives up control
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):

        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0]) # target coords for cmd_position function
        self.all_waypoints = [] # waypoints are set in calculate_box() method
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)



    def local_position_callback(self):

        """
        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        We need to check if we have reached any of the state change triggers and react as follows:
        1. If taking off, transition to WAYPOINT once desired height is reached
        2. If flying waypoints, transition to LANDING once final waypoint is reached
        3. If landing, transition to DISARMING once drone lands
        """

        if self.flight_state == States.TAKEOFF:
            # check if drone altitude is within 5% of target altitude
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                # if so, then transition to first waypoint, otherwise do nothing
                self.flight_state == States.WAYPOINT
                self.waypoint_transition()
        
        elif self.flight_state == States.WAYPOINT:
            # check to see if drone is within 1m of the target x,y coords using norm of relative position vector
            if  np.linalg.norm(self.target_position[:2] - self.local_position[:2]) < 1.0:
                # if more waypoints exist, transition to the next waypoint
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                # otherwise, tranisiton to LANDING if the drones speed is less than 1m/sec
                elif np.linalg.norm(self.local_velocity[:2]) < 1.0:
                    self.landing_transition()



    def velocity_callback(self):
        
        pass  # we do not want to trigger the disarming transition in the Aero

        """
        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        It is only used duing landing to determine when the drone GPS height is within 0.1m of the ground
         

        if self.flight_state == States.LANDING:

            if abs(self.global_position[2] - self.global_home[2]) < 0.1:
                if abs(self.local_position[2]) < 0.1:
                    self.disarming_transition()
        """


    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                # now just passively waiting for the pilot to change these attributes
                # once the pilot changes, need to update our internal state
                if self.guided:
                    self.flight_state = States.ARMING
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.LANDING:
                # check if the pilot has changed the armed and control modes
                # if so (and the script no longer in control) stop the mission
                if not self.armed and not self.guided:
                    self.stop()
                    self.in_mission = False
            elif self.flight_state == States.DISARMING:
                # no longer want the vehicle to handle the disarming and releasing control
                # that will be done by the pilot
                pass



    def calculate_box(self):
        print("Setting Home")
        cp = np.array([self.local_position[0], self.local_position[1], -self.local_position[2]])  # get the current local position -> note we need to change the sign of the down coordinate to be altitude
        local_waypoints = [cp + [10.0, 0.0, 3.0], cp + [10.0, 10.0, 3.0], cp + [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
        return local_waypoints



    def arming_transition(self):

        print("arming transition")

        # Check if the global position is still equal to zero
        if self.global_position[0] == 0.0 and self.global_position[1] == 0.0: 
            print("no global position data, wait")
            return

        # 1. Take control of the drone
        self.take_control()
        # 2. Pass an arming command
        self.arm()
        # 3. Set the home location to current position
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        # 4. Transition to the ARMING state
        self.flight_state = States.ARMING


    def takeoff_transition(self):

        print("takeoff transition")

        target_altitude = 3.0

        self.target_position[2] = target_altitude
        # 2. Command a takeoff to 3.0m
        self.takeoff(target_altitude)
        # 3. Transition to the TAKEOFF state
        self.flight_state = States.TAKEOFF


    def waypoint_transition(self):

        # 1. Pop the next waypoint position off the list and set as target
        self.target_position = self.all_waypoints.pop(0)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)

        # 2. Transition to (or remain in) WAYPOINT state
        self.flight_state = States.WAYPOINT

        print("waypoint transition to target: ", self.target_position)


    def landing_transition(self):

        print("landing transition")
        # 1. Command the drone to land at its current local position
        self.land()
        # 2. Transition to the LANDING state
        self.flight_state = States.LANDING



    def disarming_transition(self):

        print("disarm transition")

         # 1. Command the drone to disarm and release control of the drone
        self.disarm()
        self.release_control()

        # 2. Transition to the DISARMING state
        self.flight_state = States.DISARMING


    def manual_transition(self):

        print("manual transition")
        # 1. Stop the connection (and telemetry log)
        self.stop()

        # 2. End the mission
        self.in_mission = False

        # 3. Transition to the MANUAL state
        self.flight_state = States.MANUAL


    def start(self):

        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")

        # self.connection.start()
        super().start()

        # note that the rest of the program executes here and you don't proceed until the connection is closed
        
        print("Closing log file")

        self.stop_log()


if __name__ == "__main__":

#    conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False, PX4=False)

    conn = MavlinkConnection('udp:192.168.1.2:14550', PX4=True, threaded=False) # insert actual IP address of wireless adapter

    drone = BackyardFlyer(conn)

    time.sleep(2)

    # intialize the waypoints list
    drone.all_waypoints = drone.calculate_box()

    drone.start()