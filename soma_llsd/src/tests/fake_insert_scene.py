import roslib
import rospy
import sys
import argparse
import os
import uuid
import tf
import std_msgs
from soma_llsd_msgs.msg import Segment,Observation,Scene
from geometry_msgs.msg import Pose,Point,Quaternion
from soma_llsd.srv import *



if __name__ == '__main__':
    insert_scene = rospy.ServiceProxy('/soma_llsd/insert_scene',InsertScene)
    update_scene = rospy.ServiceProxy('/soma_llsd/update_scene',UpdateScene)
    get_scene = rospy.ServiceProxy('/soma_llsd/get_scene',GetScene)

    # string episode_id
    # string waypoint
    # string meta_data
    # uint32 timestamp

    # tf/tfMessage transform
    # sensor_msgs/PointCloud2 cloud
    # #sensor_msgs/Image rgb_img
    # sensor_msgs/Image depth_img
    # sensor_msgs/CameraInfo camera_info
    # geometry_msgs/Pose robot_pose

    #scene = insert_scene(None,None,None,None,None,None,None,None,None,None)
    scene = get_scene("1b206dca-b19c-4557-b471-daab1452b4bf2d")
    if(scene.response is True):
        print("success!")
        #print("scene id: " + scene.response.id)
        #print("scene waypoint: " + scene.response.waypoint)
        #scene.response.waypoint = "My House"
        #update_scene(scene.response)
        #print("scene waypoint: " + scene.response.waypoint)

    else:
        print("failure!")
