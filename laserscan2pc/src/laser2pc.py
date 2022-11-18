#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection
class Laser2PC():
    def __init__(self):
        self.laserProj=LaserProjection()
        self.pcPub=rospy.Publisher("/filtered_cloud", pc2, queue_size=1)
        self.laserSub=rospy.Subscriber("/scan",LaserScan,self.laserCallback1)

      
       


    def laserCallback1(self,data):
        cloud_out=self.laserProj.projectLaser(data)
        self.pcPub.publish(cloud_out)
    

if __name__=='__main__':
    rospy.init_node("laser2PointCloud")
    l2pc=Laser2PC()
    rospy.spin()

