#!/usr/bin/env python2

import rospy
from std_msgs.msg import *
from sensor_msgs.msg import *

import amqp_common
from rosconversions import ros_msg_to_dict


class FrontSonarBridge(object):
    """TODO!"""

    def __init__(self):
        self.ros_topic = '/sensors/range/sonar/front'
        self.amqp_exchange = 'amq.topic'
        self.amqp_topic = 'sensors.range.sonar.front'
        self.amqp_topic_namespace = 'robot_1'
        self.amqp_broker_ip = '155.207.33.185'
        self.amqp_broker_port = '5672'
        self.username = 'robot_1'
        self.password = 'r0b0t1'
        self.ros_message_type = Range
        self.ros_node_name = self.__class__.__name__

    def run(self):
        """Start the Bridge"""
        self._init_ros_subscriber()
        self._init_platform_publisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    def _init_ros_subscriber(self):
        rospy.init_node(self.ros_node_name)
        print('[*] - ROS Subscriber <{}> ready!'.format(self.ros_topic))
        rospy.Subscriber(self.ros_topic, self.ros_message_type,
                         self._ros_callback)

    def _ros_callback(self, msg):
        data = ros_msg_to_dict(msg)
        self._publish(data)

    def _init_platform_publisher(self):
        topic = '{}.{}'.format(self.amqp_topic_namespace, self.amqp_topic)
        self.pub = amqp_common.PublisherSync(
            topic,
            connection_params=amqp_common.ConnectionParameters(
                host=self.amqp_broker_ip,
                port=self.amqp_broker_port),
            creds=amqp_common.Credentials(self.username,
                                          self.password))
        print('[*] - AMQP Publisher <{}> ready!'.format(topic))

    def _publish(self, data):
        if not isinstance(data, dict):
            raise TypeError('data should be of type dict')
        self.pub.publish(data)
        print('[*] - Sent message: {}'.format(data))


if __name__ == '__main__':
    br = FrontSonarBridge()
    br.run()
