#!/usr/bin/env python2

import rospy
from {{ bridge.rosMessagePackage }}.msg import {{ bridge.rosMessageType }}

import amqp_common
from rosconversions import ros_msg_to_dict


class {{ bridge.name }}Bridge(object):
    """TODO!"""

    def __init__(self):
        self.ros_topic = "{{ bridge.rosTopic }}"
        self.amqp_exchange = "{{ bridge.amqpExchange }}"
        self.amqp_topic = "{{ bridge.amqpTopic }}"
        self.amqp_topic_namespace = "{{ bridge.amqpTopicNamespace }}"
        self.amqp_broker_ip = "{{ amqp_broker.ip }}"
        self.amqp_broker_port = "{{ amqp_broker.port }}"
        self.username = "{{ amqp_broker.username }}"
        self.password = "{{ amqp_broker.password }}"
        self.ros_message_type = {{ bridge.rosMessageType }}
        self.ros_node_name = self.__class__.__name__

    def run(self):
        """Start the Bridge"""
        self._init_ros_subscriber()
        self._init_platform_publisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)

    def _init_ros_subscriber(self):
        rospy.init_node(self.ros_node_name)
        rospy.loginfo('ROS Subscriber <{}> ready!'.format(self.ros_topic))
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
        rospy.loginfo('AMQP Publisher <{}> ready!'.format(topic))

    def _publish(self, data):
        if not isinstance(data, dict):
            raise TypeError('data should be of type dict')
        self.pub.publish(data)
        rospy.loginfo('Publishing message: {}'.format(data))


if __name__ == '__main__':
    br = {{ bridge.name }}Bridge()
    br.run()
