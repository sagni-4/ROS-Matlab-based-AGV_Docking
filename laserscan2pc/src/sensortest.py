#! /usr/bin/env python3 
import rospy 
from sensor_msgs.msg import LaserScan 
 
def callback(msg): 
# accessing data in the message for further computation
      if msg.header.frame_id=="link_1":
         print("distance from front_range_sensor1")
         front_distance1=msg.ranges[1]
         print(front_distance1)
      elif msg.header.frame_id=="link_2":
         print("distance from front_range_sensor2")
         front_distance2=msg.ranges[1]
         print(front_distance2)
      elif msg.header.frame_id=="link_3":
         print("distance from left_side_range_sensor1")
         Lside_distance1=msg.ranges[1]
         print(Lside_distance1)
      elif msg.header.frame_id=="link_4":
         print("distance from left_side_range_sensor2")
         Lside_distance2=msg.ranges[1]
         print(Lside_distance2)
      
rospy.init_node('Rangescanner_values') 
sub = rospy.Subscriber('/range/lasers', LaserScan, callback) 
loop_hz=rospy.Rate(10)
while not rospy.is_shutdown():
    loop_hz.sleep()
