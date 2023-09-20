
# https://github.com/h2r/pidrone_project3_pid/tree/d16297596fe988c2ecb7225bf3c1e0f61ff7b165

import rospy
from sensor_msgs.msg import Range
import numpy as np


class PID:
    """
    This is your PID class! Be sure to read the docstrings carefully and fill in all methods of this class!
    """

    def __init__(self, kp, ki, kd, k):
        """
        Here is where you will initialize all of the instance variables you might need!

        **IMPORTANT** Be sure to follow the following naming conventions for your control term variables:

        P term: _p
        I term: _i
        D term: _d

        This will ensure that your class works correctly with the rest of the drone's code stack.

        :param kp: The proportional gain constant
        :param ki: The integral gain constant
        :param kd: The derivative gain constant
        :param k: The offset constant that will be added to the sum of the P, I, and D control terms
        """

        self._p = 1.5
        self._i = 0.5
        self._d = 1
        self._k = k  # not used


        self.b=[9173000,-18130000,8959000] # controller coefficients b0 through bn
        self.a=[1,-1.759,.7595] # controller coefficients a1 through an

        # limit of pid loop centered around nonlinear calculated offset
        self._lowLimit = -100
        self._highLimit = 100

        self._lasterr = 0
        self._dfilter = 0
        self._sumError = 0
        self._saturationerror = 0

        self._range = 0

        rospy.Subscriber('/pidrone/range', Range, self.range_callback)

    def range_callback(self, data):
        self._range = data.range

    def step(self, err, dt):
        """
        This method will get called at each sensor update, and should return
        a throttle command output to control the altitude of the drone. This is where
        your actual PID calculations will occur. You should implement the discrete version
        of the PID control function.

        :param err: The current error (difference between the setpoint and drone's current altitude) in meters.
                    For example, if the drone was 10 cm below the setpoint, err would be 0.1
        :param dt: The time (in seconds) elapsed between measurements of the process variable (the drone's altitude)
        :returns: You should restrict your output to be between 1100 and 1900. This is a PWM command, which will be
                  sent to the SkyLine's throttle channel
        """
        """
        err = err


        a = .3  # smooths derivative, larger=less smoothing
        self._dfilter = self._dfilter*(1-a)+((err-self._lasterr)/dt)*(a)

        print("sat error", self._saturationerror)

        kaw = 1
        self._sumError += (err + kaw*self._saturationerror)*dt
        i = self._i*(self._sumError)

        print("sum error", self._sumError)

        output = self._p*err + self._d*self._dfilter + i

        print("unclipped output", output)

        _saturationerror = 0
        unclippedoutput=output
        if(output > self._highLimit):
            output = self._highLimit
        if(output < self._lowLimit):
            output = self._lowLimit
        self._saturationerror=output-unclippedoutput

        A = -204.3
        C = 6.278
        B = 1489

        print("offset", A*np.exp(-C*self._range)+B)

        output += A*np.exp(-C*self._range)+B

        self._lasterr = err

        return output
        """

        print("-------")
        print("error", err)
        print("dt", dt)

        for i in range(self.bn-1, 0, -1): # shift data right one step, moving right to left
            self.ubuf[i]=self.ubuf[i-1]
        for i in range(self.an, 0, -1):
            self.ybuf[i]=self.ybuf[i-1]
        self.ubuf[0] = self.u
        self.ybuf[0] = 0 # y(k) will be set equal to the difference equation in the following lines
        for i in range(0, self.bn):
            self.ybuf[0]+=self.b[i]*self.ubuf[i]
        for i in range(1, self.an+1):
            self.ybuf[0]-=self.a[i-1]*self.ybuf[i]a

        output = self.ybuf[0] # y(k) from difference equation
        print("output", output)
        if(output>1500):
            output=1500
        return output


    def reset(self):
        """
        This method will get called when the simulation is reset (by pressing 'r') or when the real drone transitions
        from armed mode to flying mode. You will want to reset the PID terms so that previously stored values will
        not affect the current calculations (think about what this entails)!
        """
        self._lasterr = 0
        self._dfilter = 0
        self._sumError = 0
        self._saturationerror = 0
