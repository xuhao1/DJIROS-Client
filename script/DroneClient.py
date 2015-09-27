__author__ = 'xuhao'
from dji_ros.msg import *
from std_msgs.msg import String, Float32, UInt8
from nav_msgs.msg import Odometry
from dji_ros.srv import *

import rospy


class DroneClient:
    """
    SubscriberPair implention a pair save topic name and it's own data type,
    so we can easily subscribe all the topic
    """

    SubscriberPair = {
        "acceleration": acc,
        "attitude_quad": attitude_quad,
        "battery_status": UInt8,
        "compass_info": compass,
        "ctrl_info": ctrl_info,
        "flight_status": UInt8,
        "gimbal_info": gimbal,
        "global_position": global_position,
        "local_position": local_position,
        "odom": Odometry,
        "rc_channels": rc_channels,
        "velocity": velocity
    }

    ServicePair = {
        "camera_action_service": camera_action,
        "drone_action_control": action,
        "drone_attitude_control": attitude,
        "gimbal_angle_control": gimbal_angle,
        "gimbal_speed_control": gimbal_speed,
        "gps_navigation_service": gps_navigation,
        "local_navigation_service": local_navigation,
        "obtain_release_control": control_manager,
        "waypoints_service": waypoints_navigation
    }

    def init_a_service(self, name, DataType):
        rospy.wait_for_service(self.NodeName + "/" + name)
        print "Wait for service {0}".format(self.NodeName + "/" + name)
        service_func = rospy.ServiceProxy(self.NodeName + "/" + name, DataType)
        self.services[name] = service_func
        print "Inited service {0}".format(name)
        pass

    def DroneSubscriber(self):
        for Topic, Type in self.SubscriberPair.iteritems():
            print "Subscribing topic : {0}/{1} with Type '{2}'".format(self.NodeName, Topic, Type)
            Callback = lambda data: self.Subscriber(Topic, Type, data)
            rospy.Subscriber(self.NodeName + "/" + Topic, Type, Callback)

    def Subscriber(self, TopicName, Type, _data):
        self.data[TopicName] = _data
        pass

    def InitServices(self):
        print "Init Services"
        self.services = dict()
        for ServiceName , ServiceType in self.ServicePair.iteritems():
            self.init_a_service(ServiceName, ServiceType)

    def Update(self,e):
        if self.AutoObtainControl:
            self.services["obtain_release_control"](1)

    def __init__(self, NodeName):
        rospy.init_node('DJIROS-Client')
        self.data = dict()
        self.NodeName = NodeName
        self.DroneSubscriber()
        self.InitServices()
        self.AutoObtainControl = True
        rospy.Timer(rospy.Duration(1), self.Update)
        self.services["obtain_release_control"](1)
        pass

    """
    Conventions
    """

    def Takeoff(self):
        self.services["drone_action_control"](4)

    def Landing(self):
        self.services["drone_action_control"](6)

    def Gohome(self):
        self.services["drone_action_control"](1)

    def FlytoLocal(self,x,y,z):
        self.services["local_navigation_service"](x,y,z)

def UnitTest():
    print "Unit Test of drone client"
    d = DroneClient("/DJI_ROS")
    d.Takeoff()
    d.FlytoLocal( 1000,1000,100 )
    rospy.spin()


if __name__ == "__main__":
    UnitTest()
