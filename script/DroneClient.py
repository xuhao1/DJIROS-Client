__author__ = 'xuhao'
from dji_ros.msg import *
from std_msgs.msg import String,Float32,UInt8
from nav_msgs.msg import Odometry

import rospy

class DroneClient:
    """
    SubscriberPair implention a pair save topic name and it's own data type,
    so we can easily subscribe all the topic
    """

    SubscriberPair = {
        "acceleration"    : "acc" ,
        "attitude_quad"   : "attitude_quad",
        "battery_status"  : "UInt8",
        "compass_info"    : "compass" ,
        "ctrl_info"       : "ctrl_info",
        "flight_status"   : "UInt8",
        "gimbal_info"     : "gimbal",
        "global_position" : "global_position",
        "local_position"  : "local_position" ,
        "odom"            : "Odometry",
        "rc_channels"     : "rc_channels",
        "velocity"        : "velocity"
    }

    def DroneSubscriber(self):
        for Topic,Type in self.SubscriberPair.iteritems():
            print "Subscribing topic : {0}/{1} with Type '{2}'".format(self.NodeName,Topic , Type)
            Callback = lambda data:self.Subscriber(Topic,Type,data)
            rospy.Subscriber(self.NodeName+"/"+Topic,eval(Type),Callback)

    def Subscriber(self,TopicName,Type,_data):
        self.data[TopicName] = _data
        pass

    def __init__(self,NodeName):
        rospy.init_node('DJIROS-Client')
        self.data = dict()
        self.NodeName = NodeName
        self.DroneSubscriber()
        pass

def UnitTest():
    print "Unit Test of drone client"
    DroneClient("/DJI_ROS")
    rospy.spin()

if __name__=="__main__":
    UnitTest()
