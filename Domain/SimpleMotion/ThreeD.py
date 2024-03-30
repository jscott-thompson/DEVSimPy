"""
	Author: J. Scott Thompson (j.scott.thompson.12@gmail.com)
	Date: 01/31/2024
	Description: Model of simple three-dimensional motion
    Depends:

    Init defaults: READY, sigma = 0.1
    intTransition responsible for updating state given most recent command
    extTransition sets update rate, accepts commands
    outputFnc packages the state as an output message to be recorded
    timeAdvance returns the time of the next event
    finish shuts down
"""
import random

from DomainInterface.DomainBehavior import DomainBehavior
from Domain.Basic.Object import Message
from enum import Enum, IntEnum, auto
from math import sqrt, cos, sin, acos

class ThreeD(DomainBehavior):
    class Status(Enum):
        READY = auto()
        FLYING = auto()
        FINISHED = auto()

    class Maneuver(Enum):
        STRAIGHT = auto()
        TURNING = auto()

    class StateComponents(IntEnum):
        t = 0
        x = 1
        y = 2
        z = 3
        vx = 5
        vy = 6
        vz = 7
        phi = 8
        theta = 9
        psi = 10
        phi_dot = 11
        theta_dot = 12
        psi_dot = 13

    def __init__(self):
        DomainBehavior.__init__(self)

        self.state = {'status': self.Status.READY,
                      'maneuver': self.Maneuver.STRAIGHT,
                      'commanded_orientation': [0.0] * 3,
                      'commanded_speed': 0.0,
                      'sigma': 0.1,
                      'physical_state': [0.0] * 14}

        self.msg = Message(None, None)

    def setInitialPhysicalState(self, msg):
        # Initial state message format:
        # t0, x0, y0, z0, vx0, vy0, vz0
        if not isinstance(msg.value, str):
            temp = str(msg.value[0]) + ', ' + str(msg.value[1]) + ', ' + str(msg.value[2]) + ', ' + str(1.0)
            temp += ', ' + str(1.0) + ', ' + str(1.0)
            msg.value = temp
        t0 = msg.time
        [x0, y0, z0, vx0, vy0, vz0] = [float(x) for x in msg.value.split(',')]
        speed0 = sqrt(pow(vx0, 2) + pow(vy0, 2))
        phi_0 = acos(vx0 / speed0)
        theta_0 = acos(vy0 / speed0)
        psi_0 = acos(vz0 / speed0)
        phi_dot_0 = 0.0
        theta_dot_0 = 0.0
        psi_dot_0 = 0.0
        self.state['physical_state'] = [t0, x0, y0, z0, vx0, vy0, vz0, phi_0, theta_0, psi_0, phi_dot_0, theta_dot_0,
                                        psi_dot_0]
        self.state['commanded_orientation'][0:2] = [theta_0, phi_0, psi_0]
        self.state['commanded_speed'] = speed0


    def processManeuverCommand(self, msg):
        # Format of a 3D maneuver command:
        # t, theta, psi, speed
        if not isinstance(msg.value, str):
            temp = str(msg.value[0]) + ', ' + str(msg.value[1]) + ', ' + str(msg.value[2]) + ', ' + str(1.0)
            msg = temp
        [t, theta, psi, speed] = [float(x) for x in msg.split(',')]
        comp = self.StateComponents
        self.state['commanded_orientation'][0] = theta
        self.state['commanded_orientation'][2] = psi
        self.state['commanded_speed'] = speed
        self.state['maneuver'] = self.Maneuver.TURNING

    def intTransition(self):
        # Propagate state
        comp = self.StateComponents
        current_state = self.state['physical_state']
        self.state['physical_state'][comp.t] += self.getSigma()
        speed = sqrt(pow(current_state[comp.vx], 2) + pow(current_state[comp.vy], 2) + pow(current_state[comp.vz], 2))
        self.state['physical_state'][comp.vx] = speed * cos(current_state[comp.theta]) * cos(current_state[comp.psi])
        self.state['physical_state'][comp.vy] = speed * sin(current_state[comp.theta]) * sin(current_state[comp.psi])
        self.state['physical_state'][comp.vz] = speed * sin(current_state[comp.theta]) * cos(current_state[comp.psi])
        self.state['physical_state'][comp.x] += self.state['physical_state'][comp.vx] * self.getSigma()
        self.state['physical_state'][comp.y] += self.state['physical_state'][comp.vy] * self.getSigma()
        self.state['physical_state'][comp.z] += self.state['physical_state'][comp.vz] * self.getSigma()
        self.state['physical_state'][comp.theta] = self.state['commanded_orientation'][0]
        self.state['physical_state'][comp.psi] = self.state['commanded_orientation'][2]
        self.state['physical_state'][comp.theta_dot] = (current_state[comp.theta] -
                                                        self.state['physical_state'][comp.theta]) / self.getSigma()
        self.state['physical_state'][comp.psi_dot] = (current_state[comp.psi] -
                                                      self.state['physical_state'][comp.psi]) / self.getSigma()
        self.state['maneuver'] = self.Maneuver.STRAIGHT

    def extTransition(self):
        # Pull data off input port
        msg = self.peek(self.IPorts[0])
    
        if self.state['status'] == self.Status.READY:
            # Set initial state from first input
            self.state['physical_state'] = self.setInitialPhysicalState(msg)
            self.state['status'] = self.Status.FLYING
            self.state['sigma'] = 0.1 # constant time step, hard-coded for now

        elif self.state['status'] == self.Status.FLYING:
            # Check if this is a shutdown message
            if msg in ['', '\n', '\r\n']:
                self.state['status'] = self.Status.FINISHED
            else:
                self.processManeuverCommand(msg)

        return

    def __str__(self):
        msg = str(self.state['physical_state'])[1:-1]
        return msg

    def outputFnc(self):
        try:
            self.msg.time = self.state['physical_state'][self.StateComponents.t]
            self.msg.value = self.__str__()
            self.poke(self.OPorts[0], self.msg)
        except:
            pass
    
    def timeAdvance(self):
        return self.state['sigma']