
# https://github.com/h2r/pidrone_project3_pid/tree/d16297596fe988c2ecb7225bf3c1e0f61ff7b165 

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

        self._sumError = 0
        self._derivError = 0
        self._lastErr = 0

        self._p = kp # not used
        self._i = ki # not used
        self._d = kd # not used
        self._k = k # not used

        self._lowLimit = 1100
        self._highLimit = 1900

        self.b=[111100, -212600, 101700] # controller coefficients b0 through bn
        self.a=[1, -0.2953, -0.7047] # controller coefficients a1 through an
    
        self.bn=len(self.b)
        self.an=len(self.a)

        self.ubuf=[]
        self.ybuf=[]

        self.u=0

        for i in range(0, self.bn):
            self.ubuf.append(0) # b1*u(k-1) through bn*u(k-n)
    
        self.ybuf.append(0) # y(k)
        for i in range(0, self.an):
            self.ybuf.append(0) # a1*(k-1) through an*y(k-n)
    


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

        self.u=err

        for i in range(self.bn-1, 0, -1): # shift data right one step, moving right to left
            self.ubuf[i]=self.ubuf[i-1]
        for i in range(self.an, 0, -1):
            self.ybuf[i]=self.ybuf[i-1]
        self.ubuf[0] = self.u

        self.ybuf[0] = 0 # y(k) will be set equal to the difference equation in the following lines
        for i in range(0, self.bn):
            self.ybuf[0]+=self.b[i]*self.ubuf[i]

        for i in range(1, self.an+1):
            self.ybuf[0]-=self.a[i-1]*self.ybuf[i]

        output = self.ybuf[0] # y(k) from difference equation

        output += 1300

        return output

    def reset(self):
        """
        This method will get called when the simulation is reset (by pressing 'r') or when the real drone transitions
        from armed mode to flying mode. You will want to reset the PID terms so that previously stored values will
        not affect the current calculations (think about what this entails)!
        """

        self._sumError = 0
        self._derivError = 0
        self._lastErr = 0
