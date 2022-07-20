from numpy import random
from random import choice
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree as ET

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

MAX_COORD= 0.5
MIN_COORD = 0.1

MAX_DIM = 0.4
MIN_DIM = 0.005

class Shape(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, mass=10, x=None, y=None, z=None):
        self.mass = mass
        self.x = self.y = self.z = None
    
    @abstractmethod
    def rand_dim(self):
        pass
    
    @abstractmethod
    def rand_pos(self):
        pass

    @abstractmethod
    def show(self):
        tree = ET.parse("shape.urdf.xacro")

        tree.find('xacro:property[@name="mass"]', namespaces).set("value", str(self.mass))
        tree.find('xacro:property[@name="xyz"]', namespaces).set("value",
                                                     str(self.x) + " " + str(self.y) + " " + str(self.z))
        tree.find('xacro:property[@name="shape_name"]', namespaces).set("value", self.__class__.__name__)
        return tree
    
    def same_len(self, dist):
        # Restrict 50/50 axis if length and width are the same
        coord_1 = choice([round(random.uniform(MIN_COORD + dist, MAX_COORD + dist), ndigits=4),
                          round(random.uniform(-MAX_COORD - dist, -MIN_COORD - dist), ndigits=4)])
        coord_2 = round(random.uniform(-MAX_COORD - dist, MAX_COORD + dist), ndigits=4)

        self.x, self.y = (coord_1, coord_2) if choice([0, 1]) else (coord_2, coord_1)

class Box(Shape):
    def __init__(self, length=0, width=0, height=0, mass=10):
        super(Box, self).__init__()
        self.length = length
        self.width = width
        self.height = height

    def rand_dim(self):
        self.length = round(random.uniform(MIN_DIM, MAX_DIM), ndigits=4)
        self.width = round(random.uniform(MIN_DIM, MAX_DIM), ndigits=4)
        self.height = round(random.uniform(MIN_DIM, MAX_DIM), ndigits=4)
        self.z = self.height / 2

    def rand_pos(self):
        # Restrict axis 50/50
        if self.width == self.length:
            super(Box, self).same_len(dist=self.width / 2)
        
        # Restrict one axis
        else:
            self.diff_len()
    
    def diff_len(self):
        self.x = round(random.uniform(-MAX_COORD - self.width / 2, MAX_COORD + self.width / 2), ndigits=4)
        
        if abs(self.x) - self.width / 2 < MIN_COORD:
            self.y =  round(random.uniform(MIN_COORD + self.length, MAX_COORD + self.length), ndigits=4) if choice([0, 1]) else round(random.uniform(-MAX_COORD - self.length, -MIN_COORD - self.length), ndigits=4)
        else:
            self.y = round(random.uniform(-MIN_COORD - self.length, MAX_COORD + self.length), ndigits=4)
    def show(self):
        tree = super(Box, self).show()
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))
        tree.find(
            'xacro:property[@name="width"]', namespaces).set("value", str(self.width))
        tree.find(
            'xacro:property[@name="height"]', namespaces).set("value", str(self.height))

        tree.write("shape.urdf.xacro")


class Sphere(Shape):
    def __init__(self, radius=0, mass=10):
        super(Sphere, self).__init__()
        self.radius = radius

    def rand_dim(self):
        self.radius = round(random.uniform(MIN_DIM / 2, MAX_DIM / 2), ndigits=4)
        self.z = self.radius

    def rand_pos(self):
        super(Sphere, self).same_len(self.radius)

    def show(self):
        tree = super(Sphere, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))

        tree.write("shape.urdf.xacro")


class Cylinder(Shape):
    def __init__(self, radius=0, length=0, mass=10):
        super(Cylinder, self).__init__()
        self.radius = radius
        self.length = length

    def diameter(self):
        return self.radius * 2

    def rand_dim(self):
        self.radius = round(random.uniform(MIN_DIM / 2, MAX_DIM / 2), ndigits=4)
        self.length = round(random.uniform(MIN_DIM / 2, MAX_DIM / 2), ndigits=4)
        self.z = self.length / 2

    def rand_pos(self):
        super(Cylinder, self).same_len(self.radius)


    def show(self):
        tree = super(Cylinder, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))

        tree.write("shape.urdf.xacro")


# Spawn and Delete Model
class sim_control_handler():
    def __init__(self):
        self.shape_model_name = "object"
        self.delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model_proxy = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


# Generate random primitive
def rand_shape():
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()

    shape.rand_pos()

    shape.show()


if __name__ == "__main__":
    rand_shape()
    
