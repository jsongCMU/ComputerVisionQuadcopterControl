# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2014 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.

#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.

"""
Script to run quadcopoter control system. Calls MATLAB to communicate with Kinect camera,
locate drone, and compute control parameter. This script takes MATLAB output and transmits
commands to quadcopter.
"""

import sys
import matlab.engine
sys.path.append("../lib")

import cflib.crtp

import logging
import time
from threading import Timer
from threading import Thread
from msvcrt import getch

import cflib.crtp
from cfclient.utils.logconfigreader import LogConfig
from cflib.crazyflie import Crazyflie

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class LoggingExample:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """
    def __init__(self, link_uri,filename):
        """ Initialize and run the example with the specified link_uri """

        # Create a Crazyflie object without specifying any cache dirs
        self._cf = Crazyflie()

        # Connect some callbacks from the Crazyflie API
        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        print "Connecting to %s" % link_uri

        print " opening file %s" % sys.path[1] + "\log" + filename
        try:
            self.file = open(sys.path[1] + "\log" + filename,'w')
        except:
            print "Unable to open log file"

        # Open the MATLAB engine and configure matlab communication data
        self.eng = matlab.engine.start_matlab()
        self.eng.addpath("C:\Users\Jaekyung Song\MATLAB\Exp",nargout=0) # change name as necessary
        print "Started Matlab engine"

        # self.CameraData = matlab.double([0,0,0])
        self.eng.startvid(nargout=0) # startvid.m
        self.StateTime = 0
        self.StartTime = 0
        self.ControlSettings = matlab.double([0,0,0,0])  # Thrust, roll, pitch, yawrate
        self.NewControlSettings = matlab.double([0,0,0,0]) # Thrust, roll, pitch, yawrate
        self.yaw = matlab.double([0.0])
        self.pitch = matlab.double([0.0])
        self.roll = matlab.double([0.0])
        self.batt = matlab.double([0.0])
        self.CurrentState = matlab.double([[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
        self.NewState = matlab.double([[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]])
        self.StartingPoint = matlab.double([0,0,0])
        self.timestep = 0

        self.MotorOn = False
        self.RunController = True

        self.is_connected = False
        print "Connecting to %s" % link_uri

        # Try to connect to the Crazyflie
        try:
             self._cf.open_link(link_uri)
             self.is_connected = True
             self.MotorOn = True
             print("connected")
        except:
            print("Unable to connect")

        # Initialize the observer
        print("Initializing state...")
        while float(self.batt[0][0])<1:
            if self.StateTime>0:
               self.timestep = time.time()-self.StateTime
            self.StateTime = time.time()
            self.NewState = self.eng.initialpositiongetter() # initialpositiongetter.m
            self.CurrentState = self.NewState
            if float(self.CurrentState[3][0]) > 0.5:
               print("Camera lost signal")
               self.MotorOn = False
            time.sleep(0.018)   # Necessary to give logging time to start



        print("Initial Position: ")
        print(self.CurrentState[1][0],self.CurrentState[1][1],self.CurrentState[1][2])
        self.StartingPoint = self.CurrentState[1]
        self.SetPosition = matlab.double([self.CurrentState[1][0],
                                          self.CurrentState[1][1],
                                          self.CurrentState[1][2]+0.50, 0])


        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        print("Starting feedback controller")
        self.StartTime = time.time()
        Thread(target=self._ramp_motors).start()


    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print "Connected to %s" % link_uri

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name="Stab", period_in_ms=15)
        #self._lg_stab.add_variable("acc.x", "float")
        #self._lg_stab.add_variable("acc.y", "float")
        #self._lg_stab.add_variable("acc.z", "float")
        self._lg_stab.add_variable("stabilizer.thrust", "float")
        self._lg_stab.add_variable("stabilizer.yaw", "float")
        self._lg_stab.add_variable("stabilizer.pitch","float")
        self._lg_stab.add_variable("stabilizer.roll","float")
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg_stab)
        if self._lg_stab.valid:
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        else:
            print("Could not add logconfig since some variables are not in TOC")

        # The definition of the logconfig can be made before connecting
        self._lg_batt = LogConfig(name="Batt", period_in_ms=100)
        self._lg_batt.add_variable("pm.vbat", "float")
        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        self._cf.log.add_config(self._lg_batt)
        if self._lg_batt.valid:
            # This callback will receive the data
            self._lg_batt.data_received_cb.add_callback(self._batt_log_data)
            # This callback will be called on errors
            self._lg_batt.error_cb.add_callback(self._batt_log_error)
            # Start the logging
            self._lg_batt.start()
        else:
            print("Could not add logconfig since some variables are not in TOC")


        # Start a timer to disconnect in 10s
        print("starting log")
        t = Timer(35, self._cf.close_link)
        t.start()

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from the log API when stabilizer data arrives"""
#        print "[%d][%s]: %s" % (timestamp, logconf.name, data)
        #self.accx = matlab.double([data['acc.x']])
        #self.accy = matlab.double([data['acc.y']])
        #self.accz = matlab.double([data['acc.z']])
        self.thrust = matlab.double([data['stabilizer.thrust']])
        self.yaw = matlab.double([data['stabilizer.yaw']])
        self.pitch = matlab.double([data['stabilizer.pitch']])
        self.roll = matlab.double([data['stabilizer.roll']])


    def _batt_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print "Error when logging %s: %s" % (logconf.name, msg)

    def _batt_log_data(self, timestamp, data, logconf):
        """Callback from the log API when battery data arrives"""
#        print "[%d][%s]: %s" % (timestamp, logconf.name, data)
        self.batt = matlab.double([data['pm.vbat']])
 #       print(str(self.batt[0][0]))

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no  Crazyflie
        at the speficied address)"""
        print "Connection to %s failed: %s" % (link_uri, msg)
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print "Connection to %s lost: %s" % (link_uri, msg)

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print "Disconnected from %s" % link_uri
        self.is_connected = False
        self.RunController = False
        self.file.close()

    def _ramp_motors(self):

        while self.RunController:
#           Update the Observer
            currtime = time.time()
            self.timestep=currtime-self.StateTime
            self.StateTime = currtime
            mtimestep = matlab.double([self.timestep])
            elapsedtime = matlab.double([currtime-self.StartTime])
            output = self.eng.amalgamfunctionforcrazyflie20(self.ControlSettings,self.CurrentState,mtimestep,self.SetPosition,self.yaw,self.batt,elapsedtime,self.StartingPoint,nargout=3) # amalgamfunctionforcrazyflie20.m
            self.ControlSettings = output[0]
            self.CurrentState = output[1]
            roll = float(self.ControlSettings[0][1])
            pitch = float(self.ControlSettings[0][2])
            yawrate = float(self.ControlSettings[0][3])
            thrust = 0
            if self.MotorOn:
                thrust = int(self.ControlSettings[0][0])
            self.ControlSettings = matlab.double([thrust,roll,pitch,yawrate])
            self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            if self.MotorOn:
                self.file.write(str(self.StateTime)+", "+                   #1
                                str(self.CurrentState[0][0])+", "+          #2
                                str(self.CurrentState[0][1])+", "+          #3
                                str(self.CurrentState[0][2])+", "+          #4
                                str(self.CurrentState[1][0])+", "+          #5
                                str(self.CurrentState[1][1])+", "+          #6
                                str(self.CurrentState[1][2])+", "+          #7
                                str(self.CurrentState[2][0])+", "+          #8
                                str(self.CurrentState[2][1])+", "+          #9
                                str(self.CurrentState[2][2])+", "+          #10
                                str(self.CurrentState[4][0])+", "+          #11
                                str(self.CurrentState[4][1])+", "+          #12
                                str(self.CurrentState[4][2])+", "+          #13
                                str(self.yaw[0][0])+", "+                   #14
                                str(pitch)+", "+                            #15
                                str(self.pitch[0][0])+","+                  #16
                                str(roll)+", "+                             #17
                                str(self.roll[0][0])+", "+                  #18
                                str(thrust)+", "+                           #19
                                str(self.thrust[0][0])+","+                 #20
                                str(self.batt[0][0]) +", "+                 #21
                                str(output[2])+                             #22
                                '\n' )

            time.sleep(0.018)

        self._cf.commander.send_setpoint(0, 0, 0, 0)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print "Scanning interfaces for Crazyflies..."
    available = cflib.crtp.scan_interfaces()
    print "Crazyflies found:"
    for i in available:
        print i[0]

    if len(available) > 0:
        le = LoggingExample(available[0][0],"\calibrationfilev2.txt")
    else:
        print "No Crazyflies found, cannot run example"

    # The Crazyflie lib doesn't contain anything to keep the application alive,
    # so this is where your application should do something. In our case we
    # are just waiting until we are disconnected.
    while le.is_connected:
        time.sleep(1)

    print "The End."
