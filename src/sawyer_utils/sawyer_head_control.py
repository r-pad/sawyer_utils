# -*- coding: utf-8 -*-
"""
Created on Sun Apr 29 14:05:28 2018

@author: bokorn
"""

import intera_interface

HEAD_LIMITS = [-5.1, 0.9]

class SawyerHeadControl(object):
    def __init__(self):
        self.head = intera_interface.Head()

    def _wrap_head_angle(self, theta):
        if theta > HEAD_LIMITS[1]:
            theta -= 2.0*np.pi
        if theta < HEAD_LIMITS[0]:
            if(self.head.pan() > sum(HEAD_LIMITS)/2.):
                theta = HEAD_LIMITS[1]
            else:
                theta = HEAD_LIMITS[0]
        return theta

    def move_head(self, delta):
        theta = self.head.pan()
        theta += delta 
        self.head.set_pan(self._wrap_head_angle(theta))

    def set_head(self, theta):
        self.head.set_pan(self._wrap_head_angle(theta)) 

