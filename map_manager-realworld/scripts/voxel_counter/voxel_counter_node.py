#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud2



def setRecordFlagTimer(event):
    print("setRecordFlagTimer")
    global recordFlag
    recordFlag = True
    

def counterCB(pc):
    global recordFlag
    seconds = rospy.get_time()
    print("counterCB")
    if (recordFlag):   
        print(f"map_size is {pc.width}")
        file = open("voxel_numbers.txt",'a+')
        file.write(f"time: {seconds} ")
        file.write(f"num of voxels: {pc.width}\n")
        file.close()
    recordFlag = False
    

    
# def counter():
#     rospy.Subscriber("/occupancy_map/explored_voxel_map",PointCloud2,counterCB)
#     pass

if __name__ == "__main__":
    global recordFlag
    global time_interval # in ms
    
    
    recordFlag = False
    time_interval = 2
    rospy.init_node("vox_counter",anonymous=True)
    # timer = rospy.Timer(rospy.Duration(2),setRecordFlagTimer)
    rospy.Subscriber("/occupancy_map/explored_voxel_map",PointCloud2,counterCB)
    rate = rospy.Rate(time_interval)
    while not rospy.is_shutdown():
        recordFlag = True
        print("setRecordFlagTimer")
        print(recordFlag)
        rate.sleep()
    rospy.spin()