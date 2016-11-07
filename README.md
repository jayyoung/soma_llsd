# SOMa Low Level Sensory Datastore

This package provides a totally optional set of helper tools for the task of composing SOMa objects from individual observations of real-world objects. The tools here, comprising a custom set of ROS messages and services associated with working with them, are intended to serve as a way of storing and manipulating sensor data such as RGB images, RGB-D clouds, image masks etc. The classic use-case being that of collecting multiple segmented observations of an object before merging them into a single object model. In this use-case, the low-level segmented observations are stored in the data structures provided by this package, and are used as the source material to create a SOMa object. However, the implementation of how these things are achieved -- data collection, filtering, generation of masks, cloud merging -- are all left up to the application developer.

# Messages

# Services

# 
