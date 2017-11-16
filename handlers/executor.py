import depot
import deviceHandler
from handlers.genericPositioner import GenericPositionerHandler
import time

## This handler is responsible for executing portions of experiments.
class ExecutorHandler(deviceHandler.DeviceHandler):
    ## callbacks must include the following:
    # - examineActions(name, table): Perform any necessary validation or
    #   modification of the experiment's ActionTable. 
    # - getNumRunnableLines(name, table, index): Given an input ActionTable, 
    #   return the number of lines in the table that we can implement, starting
    #   from the specified index.
    # - executeTable(name, table, startIndex, stopIndex): Actually perform
    #   actions through the specified lines in the ActionTable.
    def __init__(self, name, groupName, callbacks):
        # Note that even though this device is directly involved in running
        # experiments, it is never itself a part of an experiment, so 
        # we pass False for isEligibleForExperiments here.
        deviceHandler.DeviceHandler.__init__(self, name, groupName, False,
                callbacks, depot.EXECUTOR)
        # Base class contains empty dicts used by mixins so that methods like
        # getNumRunnableLines can be implemented here for all mixin combos.
        self.digitalClients = {}
        self.analogClients = {}
        if not isinstance(self, DigitalMixin):
            self.registerDigital = self._raiseNoDigitalException
            self.getDigital = self._raiseNoDigitalException
            self.setDigital = self._raiseNoDigitalException
            self.triggerDigital = self._raiseNoDigitalException
        if not isinstance(self, AnalogMixin):
            self.registerAnalog = self._raiseNoAnalogException
            self.setAnalog = self._raiseNoAnalogException
            self.getAnalog = self._raiseNoAnalogException
            self.setAnalogClient = self._raiseNoAnalogException
            self.getAnalogClient = self._raiseNoAnalogException

    def examineActions(self, table):
        return self.callbacks['examineActions'](self.name, table)

    def getNumRunnableLines(self, table, index):
        ## Return number of lines this handler can run.
        count = 0
        for time, handler, parameter in table[index:]:
            # Check for analog and digital devices we control.
            if (handler is not self and
                   handler not in self.digitalClients and
                   handler not in self.analogClients):
                # Found a device we don't control.
                break
            count += 1
        return count

    def _raiseNoDigitalException(self, *args, **kwargs):
        raise Exception("Digital lines not supported.")

    def _raiseNoAnalogException(self, *args, **kwargs):
        raise Exception("Analog lines not supported")

    ## Run a portion of a table describing the actions to perform in a given
    # experiment.
    # \param table An ActionTable instance.
    # \param startIndex Index of the first entry in the table to run.
    # \param stopIndex Index of the entry before which we stop (i.e. it is
    #        not performed).
    # \param numReps Number of times to iterate the execution.
    # \param repDuration Amount of time to wait between reps, or None for no
    #        wait time. 
    def executeTable(self, table, startIndex, stopIndex, numReps, repDuration):
        return self.callbacks['executeTable'](self.name, table, startIndex, 
                stopIndex, numReps, repDuration)


class DigitalMixin(object):
    ## Digital handler mixin.

    ## Register a client device that is connected to one of our lines.
    def registerDigital(self, client, line):
        self.digitalClients[client] = line

    def setDigital(self, line, state):
        self.callbacks['setDigital'](line, state)

    def triggerDigital(self, client, dt=0.01):
        ## Trigger a client line now.
        line = self.digitalClients.get(client, None)
        if line:
            self.setDigital(line, True)
            time.sleep(dt)
            self.setDigital(line, False)


class AnalogMixin(object):
    ## Analog handler mixin.
    # Consider output 'level' in volts, amps or ADUS, and input
    # 'position' in experimental units (e.g. um or deg).
    # level = gain * (offset + position)
    # gain is in units of volts, amps or ADUS per experimental unit.
    # offset is in experimental units.

    def registerAnalog(self, client, line, offset=0, gain=1, movementTimeFunc=None):
        ## Register a client device that is connected to one of our lines.
        # Returns an AnalogLineHandler for that line.
        h = AnalogLineHandler(
            client.name, self.name + ' analogs',
            {'moveAbsolute': lambda pos: self.setClientPosition(h, pos),
             'getPosition': lambda: self.getClientPosition(h),
             'getMovementTime': movementTimeFunc,})
        self.analogClients[h] = (int(line), float(offset), float(gain))
        return h

    def setAnalogLine(self, line, level):
        ## Set analog output of line to level.
        self.callbacks['setAnalog'](line, level)

    def getAnalogLine(self, line):
        ## Get level of analog line.
        return self.callbacks['getAnalog'](line)

    def setClientPosition(self, client, value):
        ## Scale a client position to an analog level and set that level.
        line, offset, gain = self.analogClients[client]
        self.callbacks['setAnalog'](line, gain * (offset + value))

    def getClientPosition(self, client):
        ## Fetch level for a client, and scale to a client position.
        line, offset, gain = self.analogClients[client]
        raw = self.callbacks['getAnalog'](line)
        return (raw / gain) - offset


class AnalogLineHandler(GenericPositionerHandler):
    ## A type of GenericPositioner for analog outputs.
    def __init__(self, name, groupName, callbacks):
        self.callbacks = callbacks
        self.callbacks['moveRelative'] = self.moveRelative
        deviceHandler.DeviceHandler.__init__(self, name, groupName, True,
                                             self.callbacks, depot.GENERIC_POSITIONER)

    def moveRelative(self, delta):
        self.callbacks['moveAbsolute'](self.callbacks['getPosition']() + delta)


class DigitalExecutorHandler(DigitalMixin, ExecutorHandler):
    pass


class AnalogExecutorHandler(AnalogMixin, ExecutorHandler):
    pass


class AnalogDigitalExecutorHandler(AnalogMixin, DigitalMixin, ExecutorHandler):
    pass
