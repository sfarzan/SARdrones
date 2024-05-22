"""

Description
-----------
Simple Implementation of the Kalman Filter for 1D data, without any dependencies
Originally written in JavaScript by Wouter Bulten
Now rewritten in Python

License
-------
MIT License
2017

Author
------
Sifan Ye

See
---
https://github.com/wouterbulten/kalmanjs
    
"""

import math

class KalmanFilter_rssi:

    cov = float('nan')
    x = float('nan')

    def __init__(self, R, Q):
        """
        Constructor

        :param R: Process Noise
        :param Q: Measurement Noise
        """
        self.A = 1
        self.B = 0
        self.C = 1

        self.R = R
        self.Q = Q

    def filter(self, measurement):
        """
        Filters a measurement

        :param measurement: The measurement value to be filtered
        :return: The filtered value
        """
        u = 0
        if math.isnan(self.x):
            self.x = (1 / self.C) * measurement
            self.cov = (1 / self.C) * self.Q * (1 / self.C)
        else:
            predX = (self.A * self.x) + (self.B * u)
            predCov = ((self.A * self.cov) * self.A) + self.R

            denominator = (self.C * predCov * self.C) + self.Q
            if denominator == 0:
                denominator += 1e-10
            # Kalman Gain
            K = predCov * self.C * (1 / (denominator));

            # Correction
            self.x = predX + K * (measurement - (self.C * predX));
            self.cov = predCov - (K * self.C * predCov);

        return self.x

    def get_cov(self):
        
        #Returns the certainity of the current_measuremet
        return self.cov

    def last_measurement(self):
        """
        Returns the last measurement fed into the filter

        :return: The last measurement fed into the filter
        """
        return self.x

    def set_measurement_noise(self, noise):
        """
        Sets measurement noise

        :param noise: The new measurement noise
        """
        self.Q = noise

    def set_process_noise(self, noise):
        """
        Sets process noise

        :param noise: The new process noise
        """
        self.R = noise
