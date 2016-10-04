import roslib
import rospy
import sys
import argparse
import os
import uuid
import tf
import std_msgs
import pymongo
from mongodb_store.message_store import MessageStoreProxy
from soma_llsd_msgs.msg import Segment,Observation,Scene
from geometry_msgs.msg import Pose,Point,Quaternion
from soma_llsd.srv import *

class StoreController():
    def __init__(self):
        rospy.init_node('soma_llsd_services', anonymous = False)
        rospy.loginfo("SOMa LLSD setting up services")
        self.scene_store = MessageStoreProxy(database="soma_llsd", collection="scene_store")
        self.segment_store = MessageStoreProxy(database="soma_llsd", collection="segment_store")
        rospy.loginfo("Done!")

        get_scene = rospy.Service('/soma_llsd/get_scene',GetScene,self.get_scene_cb)
        insert_scene = rospy.Service('/soma_llsd/insert_scene',InsertScene,self.insert_scene_cb)
        update_scene = rospy.Service('/soma_llsd/update_scene',UpdateScene,self.update_scene_cb)
        #insert_scene_auto - TODO


        get_segment = rospy.Service('/soma_llsd/get_segment',GetSegment,self.get_segment_cb)
        insert_segment = rospy.Service('/soma_llsd/insert_segment',InsertSegment,self.insert_segment_cb)
        add_observations_to_segment = rospy.Service('/soma_llsd/add_observations_to_segment',AddObservationsToSegment,self.add_obs_cb)


        rospy.spin()

    def add_obs_cb(self,req):
        rospy.loginfo("-- Request to add observations to segment")
        b = self.add_observations_to_segment(req.segment_id,req.observations)
        result = AddObservationsToSegmentResponse(b)
        return result

    def insert_segment_cb(self,req):
        rospy.loginfo("-- Request to insert segment recieved")
        b,r = self.insert_segment(req.meta_data,req.scene_id,req.observations)
        result = InsertSegmentResponse(b,r)
        return result

    def get_segment_cb(self,req):
        r,s = self.get_segment(req.segment_id)
        result = GetSegmentResponse(r,s)
        return result

    def get_scene_cb(self,req):
        r,s = self.get_scene(req.scene_id)
        result = GetSceneResponse(r,s)
        return result

    def update_scene_cb(self,req):
        rospy.loginfo("-- Request to update scene recieved")
        sc = self.update_scene(req.input)
        result = UpdateSceneResponse(sc)
        return result

    def insert_scene_cb(self,req):
        rospy.loginfo("-- Request to insert scene recieved")
        b,r = self.insert_scene(req.episode_id,
        req.waypoint,
        req.meta_data,
        req.timestamp,
        req.transform,
        req.cloud,
        req.rgb_img,
        req.depth_img,
        req.camera_info,
        req.robot_pose)
        result = InsertSceneResponse(b,r)
        return result

    def insert_scene_auto(self,req):
        # get the messages from topics
        sc = self.insert_scene(req.episode_id,
        req.waypoint,
        req.meta_data,
        req.timestamp,
        req.transform,
        req.cloud,
        req.rgb_img,
        req.depth_img,
        req.camera_info,
        req.robot_pose)
        result = InsertSceneResponse(sc)

        return result

    def insert_scene(self,episode_id,waypoint,meta_data,timestamp,tf,cloud,rgb_img,depth_img,camera_info,robot_pose):
        try:
            new_scene = Scene()
            new_scene.id = str(uuid.uuid4())
            new_scene.episode_id = episode_id
            new_scene.waypoint = waypoint
            new_scene.meta_data = meta_data
            new_scene.timestamp = timestamp
            new_scene.transform = tf
            new_scene.cloud = cloud
            new_scene.rgb_img = rgb_img
            new_scene.depth_img = depth_img
            new_scene.camera_info = camera_info
            new_scene.robot_pose = robot_pose
            self.scene_store.insert_named(new_scene.id,new_scene)
            rospy.loginfo("-- Scene successfully inserted with id " + new_scene.id)
            return True,new_scene
        except Exception,e:
            rospy.loginfo(e)
            return False,Scene()

    def get_scene(self,scene_id):
        scene,meta = self.scene_store.query_named(scene_id, Scene._type)
        if(not scene):
            print("Unable to find scene with ID: " + scene_id)
            return False,None
        return True,scene

    def update_scene(self,scene):
        try:
            self.scene_store.update_named(scene.id, scene)
            rospy.loginfo("-- Scene successfully updated")
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

    def insert_segment(self,meta_data,scene_id,observations):
        try:
            new_segment = Segment()
            new_segment.id = str(uuid.uuid4())
            #new_segment.timestamp = scene.timestamp
            new_segment.meta_data = meta_data
            new_segment.scene_id = scene_id
            new_segment.observations = observations
            self.segment_store.insert_named(new_segment.id,new_segment)
            rospy.loginfo("-- Success! added segment " + new_segment.id)
            return True,new_segment
        except Exception,e:
            rospy.loginfo(e)
            return False,Segment()

    def get_segment(self,segment_id):
        segment,meta = self.segment_store.query_named(segment_id, Segment._type)
        if(not segment):
            print("Unable to find segment with ID: " + scene_id)
            return None
        return segment

    def update_segment(self,segment):
        try:
            self.segment_store.update_named(segment.id,segment)
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False

    def add_observations_to_segment(self,segment_id,observations):
        try:
            segment = self.get_segment(segment_id)
            if(not segment):
                rospy.loginfo("Unable to find segment with ID: " + segment_id)
                return False

            for o in observations:
                segment.observations.append(o)

            self.segment_store.update_named(segment_id,segment)
            return True
        except Exception,e:
            rospy.loginfo(e)
            return False


if __name__ == '__main__':
#    rospy.loginfo("SOMa LLSD Started")
    c = StoreController()

    #c.insert_scene()
    #c.insert_segment()

    #o_a = Observation()
    #o_b = Observation()

    #c.append_observations_to_segment("3165ae89-7728-4444ca7-98c2-c8289cee995f",[o_a,o_b])
