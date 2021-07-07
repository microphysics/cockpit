#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2018 Mick Phillips <mick.phillips@gmail.com>
## Copyright (C) 2018 Ian Dobbie <ian.dobbie@bioch.ox.ac.uk>
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

## Copyright 2013, The Regents of University of California
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## 1. Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## 3. Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived
##   from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
## LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
## ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.


from cockpit import events
import cockpit.gui.guiUtils
import cockpit.handlers.stagePositioner
from cockpit import interfaces
import cockpit.util.logger
import cockpit.util.threads
import cockpit.util.userConfig

from threading import Lock

from cockpit.devices.device import Device
from OpenGL.GL import *
import serial
import threading
import time
import wx
import re # to get regular expression parsing for config file

import sys
import os
#import cockpit.drivers.smaractctl.smaract.ctl.bindings
import smaract.ctl as ctl
VELOCITY = 25000000000
ACCELERATION = 250000000000
scale = 1000000



## This module is for Smaract MCS2-controlled XYZ stage.
# Sample config entry:
#  [smaractXYZ]
# incomplete
#  softlimits: ((-15000, -15000, -15000), (15000, 15000, 15000))
#

LIMITS_PAT = r"(?P<limits>\(\s*\(\s*[-]?\d*\s*,\s*[-]?\d*\s*\)\s*,\s*\(\s*[-]?\d*\s*\,\s*[-]?\d*\s*\)\))"

## TODO:  Test with hardware.
## TODO:  These parameters should be factored out to a config file.

d_handle = int
d_handle = 0
d_handle = int(d_handle)

positionUpdateThreadExistsLock = Lock()
xyzMotionTargetsLock = Lock()

class SmaractXYZ(Device):
    def __init__(self, name, config):
        super().__init__(name, config)
        ## Connection to the XYZ stage controller (serial.Serial instance)
        self.xyzConnection = None
        ## Lock around sending commands to the XYZ stage controller.
        self.xyzLock = threading.Lock()
        ## Cached copy of the stage's position. Initialized to an impossible
        # value; this will be modified in initialize.
        self.xyzPositionCache = (10 ** 100, 10 ** 100, 10 ** 100)
        ## Target positions for movement in X, Y and Z, or None if that ch is 
        # not moving.
        self.xyzMotionTargets = [None, None, None]

        ## If there is a config section for the smaractMCS2, grab the config and
        # subscribe to events.
        try :
            limitString = config.get('softlimits')
            parsed = re.search(LIMITS_PAT, limitString)
            if not parsed:
                # Could not parse config entry.
                raise Exception('Bad config: smaractMCS2XYZ Limits.')
                # No transform tuple
            else:
                lstr = parsed.groupdict()['limits']
                self.softlimits=eval(lstr)
        except:
            print ("No softlimits section setting default limits")
            self.softlimits = ((-5000, -5000, -3000), (5000, 5000, 15000))

        events.subscribe(events.USER_ABORT, self.onAbort)
        #events.subscribe('macro stage xyz draw', self.onMacroStagePaint)
        #events.subscribe('cockpit initialization complete', self.promptExerciseStage)


    def initialize(self):
        try:
            buffer = ctl.FindDevices("")
            if len(buffer) == 0:
                print("MCS2 no devices found.")
                sys.exit(1)
            locators = buffer.split("\n")
            for locator in locators:
                print("MCS2 available devices: {}".format(locator))
        except:
            print("MCS2 failed to find devices. Exit.")
            input()
            sys.exit(1)

        try:
            # Open the first MCS2 device from the list
            global d_handle
            d_handle = ctl.Open(locators[0])

            print("MCS2 opened {}.".format(locators[0]))
                # Get the proper initial position.
            self.getXYZPosition()
            for ch in range(2):
                move_mode = ctl.MoveMode.CL_ABSOLUTE
            self.findReference()

            
        except ctl.Error as e:
        # Passing an error code to "GetResultInfo" returns a human readable string
        # specifying the error.
            print("MCS2 {}: {}, error: {} (0x{:04X}) in line: {}.".format(e.func, ctl.GetResultInfo(e.code), ctl.ErrorCode(e.code).name, e.code, (sys.exc_info()[-1].tb_lineno)))

        except Exception as ex:
            print("Unexpected error: {}, {} in line: {}".format(ex, type(ex), (sys.exc_info()[-1].tb_lineno)))
            raise
        
        # start stage movement event loop
        
        
        
    ## Home the motors.
    def findReference(self):
        for ch in reversed(range(3)):
            r_id = ctl.RequestReadProperty(d_handle, ch, ctl.Property.CHANNEL_STATE, 0)
            state = ctl.ReadProperty_i32(d_handle, r_id)
            if (state & ctl.ChannelState.IS_CALIBRATED) == 0: #not calibrated (please do this by manually)
                cockpit.gui.guiUtils.showHelpDialog(None, 'Channel '+ch+' is NOT CALIBRATED)')
                sys.exit(1)
            if (state & ctl.ChannelState.IS_REFERENCED) == 0:
                busy_box = wx.ProgressDialog(parent = None, title = 'Busy...', message = 'Homing stage ch '+str(ch))
                busy_box.Show()
                ctl.Reference(d_handle, ch)
                while (ctl.GetProperty_i32(d_handle, ch, ctl.Property.CHANNEL_STATE) & ctl.ChannelState.ACTIVELY_MOVING != 0):
                    time.sleep(0.1)
                busy_box.Hide()
                busy_box.Destroy()

                # Was homing successful?
                msg = ''
                r_id = ctl.RequestReadProperty(d_handle, ch, ctl.Property.CHANNEL_STATE, 0)
                state = ctl.ReadProperty_i32(d_handle, r_id)
                if (state & ctl.ChannelState.IS_REFERENCED) == 0:
                    msg += 'There was a problem homing ch '+str(ch)+'.\n'
                    cockpit.gui.guiUtils.showHelpDialog(None, msg)
                    return(1)
                else:
                    self.sendXYZPositionUpdates()
                    cockpit.gui.guiUtils.showHelpDialog(None, 'Homing successful.')
        return(0)


    ## When the user logs out, switch to open-loop mode.
    def onExit(self):
        # Switch to open loop?
        self.xyzConnection.close()


    ## Halt XYZ motion when the user aborts. Note we can't control Z motion
    # here because the piezo is under the DSP's control.
    def onAbort(self, *args):
        ctl.Stop(d_handle, 2)
        ctl.Stop(d_handle, 1)
        ctl.Stop(d_handle, 0)
        ctl.Stop(d_handle, 2)
        ctl.Stop(d_handle, 1)
        ctl.Stop(d_handle, 0)


    def getHandlers(self):
        result = []
        # NB these motion limits are more restrictive than the stage's true
        # range of motion, but they are needed to keep the stage from colliding
        # with the objective. 
        for ch, minPos, maxPos in [(0, self.softlimits[0][0],self.softlimits[1][0]),
                    (1, self.softlimits[0][1],self.softlimits[1][1]), (2, self.softlimits[0][2],self.softlimits[1][2])]:
            result.append(cockpit.handlers.stagePositioner.PositionerHandler(
                    "%d SmaractMover" % ch, "%d stage motion" % ch, False,
                    {'moveAbsolute': self.moveXYZAbsolute,
                        'moveRelative': self.moveXYZRelative,
                        'getPosition': self.getXYZPosition},
                    ch, (minPos, maxPos), (minPos, maxPos)))
        return result


    def moveXYZAbsolute(self, ch, pos):
        #print("moveXYZAbsolute " + str(ch) + " " + str(pos))
        with self.xyzLock:
            if self.xyzMotionTargets[ch] != None:
                # Don't stack motion commands for the same ch
                return
        
        #this should be set every time, as it may help with controling the stage via the handheld module and cockpit at the same time (according to manufacturer)
        ctl.SetProperty_i32(d_handle, ch, ctl.Property.MOVE_MODE, ctl.MoveMode.CL_ABSOLUTE)
        ctl.SetProperty_i64(d_handle, ch, ctl.Property.MOVE_VELOCITY, VELOCITY)
        ctl.SetProperty_i64(d_handle, ch, ctl.Property.MOVE_ACCELERATION, ACCELERATION)
        # The factor of 10000000 converts from µm to pm.
        #print('MOVING ' + str(ch) + ' ' + str(self.xyzMotionTargets[ch]))
        
        xyzMotionTargetsLock.acquire()
        if self.xyzMotionTargets == [None, None, None]:
            self.sendXYZPositionUpdates()              
        self.xyzMotionTargets[ch] = pos
        ctl.Move(d_handle, ch, int(pos * scale))
        self.getXYZPosition(ch)
        events.publish(events.STAGE_MOVER, ch) 
        xyzMotionTargetsLock.release()

    def moveXYZRelative(self, ch, delta):
        if not delta:
            # Received a bogus motion request.
            return
        #print("moveXYZRelative " + str(ch) + " " + str(delta))
        curPos = self.xyzPositionCache[ch]
        self.moveXYZAbsolute(ch, curPos + delta)


    ## Send updates on the XYZ stage's position, until it stops moving.
    #@cockpit.util.threads.callInNewThread
    #def sendXYZPositionUpdates(self):
    #   while True:
            #time.sleep(0.1)
            
            #xyzMotionTargetsLock.acquire()
            #for ch in range(3):
                #if self.xyzMotionTargets[ch] is not None:
                    ##if (ctl.GetProperty_i32(d_handle, ch, ctl.Property.CHANNEL_STATE) & ctl.ChannelState.ACTIVELY_MOVING == 0):
                        ##self.xyzMotionTargets[ch] = None
                        ##events.publish(events.STAGE_STOPPED, '%d SmaractMover' % ch)
                        ###print('stopped %d\n' % ch)
                    ##else:
                        ##print('xyzMT '+str(ch)+' '+str(self.xyzMotionTargets))
                        #self.getXYZPosition(ch)
                        #events.publish(events.STAGE_MOVER, ch)
            #xyzMotionTargetsLock.release();
                       
            #if self.xyzMotionTargets is [None, None, None]: #TODO: set these to None via waitForEvent(). I assume this has to be done in a separate thread.
                #return
            
    @cockpit.util.threads.callInNewThread        
    def sendXYZPositionUpdates(self):
        timeout = 100 # in ms
        
        print("Movement event waiter started");
        
        # loop as long as any channel still has a target
        while True:
                                   
            try:
                event = ctl.WaitForEvent(d_handle, timeout)
                # The "type" field specifies the event.
                # The "idx" field holds the device index for this event, it will always be "0", thus might be ignored here.
                # The "i32" data field gives additional information about the event.
                if event.type == ctl.EventType.MOVEMENT_FINISHED:
                    if (event.i32 == ctl.ErrorCode.NONE):
                        # Movement finished.
                        xyzMotionTargetsLock.acquire()
                        self.xyzMotionTargets[event.idx] = None
                        self.getXYZPosition()
                        events.publish(events.STAGE_MOVER, event.idx) 
                        events.publish(events.STAGE_STOPPED, '%d SmaractMover' % event.idx) 
                        print("MCS2 movement finished, channel: ", event.idx)                   
                        xyzMotionTargetsLock.release()
                    else:
                        # The movement failed for some reason. E.g. an endstop was detected.
                        # TODO: this should raise an error conditions
                        print("MCS2 movement finished, channel: {}, error: 0x{:04X} ({}) ".format(event.idx, event.i32, ctl.GetResultInfo(event.i32)))
                else:
                    # The code should be prepared to handle unexpected events beside the expected ones.
                    # TODO: this should raise an error conditions
                    print("MCS2 received event: {}".format(ctl.GetEventInfo(event)))

            except ctl.Error as e:
                if e.code == ctl.ErrorCode.TIMEOUT:
                    print("Info: MCS2 wait for event timed out after {} ms".format(timeout))
                    for ch in range(3):
                        xyzMotionTargetsLock.acquire()
                        if ((self.xyzMotionTargets[ch] != None) and (ctl.GetProperty_i32(d_handle, ch, ctl.Property.CHANNEL_STATE) & ctl.ChannelState.ACTIVELY_MOVING == 0)):
                            self.xyzMotionTargets[ch] = None
                            self.getXYZPosition()
                            events.publish(events.STAGE_MOVER, event.idx) 
                            events.publish(events.STAGE_STOPPED, '%d SmaractMover' % ch) 
                            print("INFO: MCS2 movement finished in timeout, channel: ", ch)                                               
                        else:
                            self.getXYZPosition()
                            events.publish(events.STAGE_MOVER, ch) 
                        xyzMotionTargetsLock.release()
                else:
                    print("MCS2 {}".format(ctl.GetResultInfo(e.code)))
                    
            xyzMotionTargetsLock.acquire()
            print( self.xyzMotionTargets )
            if self.xyzMotionTargets == [None, None, None]:
                print("Movement event waiter finished")
                xyzMotionTargetsLock.release()
                return
            xyzMotionTargetsLock.release()
                    

    ## Get the position of the specified ch, or both axes by default.
    def getXYZPosition(self, ch = None):
        # Positions are in pm, and we need µm.
        x = ctl.GetProperty_i64(d_handle, 0, ctl.Property.POSITION) /1000000
        y = ctl.GetProperty_i64(d_handle, 1, ctl.Property.POSITION) /1000000
        z = ctl.GetProperty_i64(d_handle, 2, ctl.Property.POSITION) /1000000
        self.xyzPositionCache = (x, y, z)
        #print(self.xyzPositionCache)
        if ch == None:
            return self.xyzPositionCache
        return self.xyzPositionCache[ch]


    def makeInitialPublications(self):
        self.sendXYZPositionUpdates()


