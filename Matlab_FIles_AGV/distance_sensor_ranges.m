function[front_Left_dist,front_right_dist,Left_side_front_dist,Left_side_rear_dist]=distance_sensor_ranges(msg1,msg2,msg3,msg4)

range1=receive(msg1);
range2=receive(msg2);
range3=receive(msg3);
range4=receive(msg4);


      front_Left_dist=range1.Ranges(1);
    
      front_right_dist=range2.Ranges(1);
    
      Left_side_front_dist=range3.Ranges(1);
    
      Left_side_rear_dist=range4.Ranges(1);
end