
import math
import numpy as np


class LaserSensor:
    def __init__(self, scans):
        self.scans=subscibetoscans
        self.position=subscribetorobotpoistion
        self.sensedobstacles=[]
        
    def distance(self,obstacleposition):
        px=(obstacleposition[0]-self.position[0])**2
        py=(obstacleposition-self.position[1])**2
        return math.sqrt(px+py)

    def ad2pos(self, distance, angle,robotposition):
        x=distance*math.cos(angle)+robotposition[0]
        y=distance*math.sin(angle)+robotposition[1]
        return(int(x), int(y))
    