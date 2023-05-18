
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
        self._k = k # not used

        # limit of pid loop centered around nonlinear calculated offset
        self._lowLimit = -100
        self._highLimit = 200

        self._lasterr = 0
        self._dfilter = 0
        self._sumError = 0
        self._saturationerror = 0


        self._range=0

        rospy.Subscriber('/pidrone/range', Range, self.range_callback)

    def range_callback(self, data):
        self._range=data.range


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

        err = err

        print("-------")
        print(err)

        a=.3 # smooths derivative, larger=less smoothing
        self._dfilter=self._dfilter*(1-a)+((err-self._lasterr)/dt)*(a)

        print(self._saturationerror)

        self._sumError += err * dt
        kaw=0.1
        i = self._i*(self._sumError+kaw*self._saturationerror)

        output=self._p*err + self._d*self._dfilter + i

        _saturationerror=0;
        if(output > self._highLimit):
            self._saturationerror=self._highLimit-output
            output=self._highLimit
        if(output < self._lowLimit):
            self._saturationerror=output-self._lowLimit
            output=self._lowLimit

        A=-204.3
        C=6.278
        B=1489

        output += A*np.exp(-C*self._range)+B

        self._lasterr=err


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


    
