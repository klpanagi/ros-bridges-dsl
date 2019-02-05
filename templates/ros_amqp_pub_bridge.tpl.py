#!/usr/bin/env python2

import rospy

{% set msgPkg = rospub.topic.msgType.split('/')[0] %}
{% set msgType = rospub.topic.msgType.split('/')[1] %}
from {{ msgPkg }}.msg import {{ msgType }}

import amqp_common
from rosconversions import ros_msg_to_dict


class {{ conn_name }}(object):
    """TODO!"""

    def __init__(self):
        self.ros_topic = "{{ rospub.topic.uri }}"
        self.amqp_exchange = "{{ amqp_topic.exchange }}"
        self.amqp_topic = "{{ amqp_topic.uri }}"
        self.amqp_topic_namespace = "{{ amqp_topic.namespace }}"
        self.amqp_broker_ip = "{{ amqp_broker.ip }}"
        self.amqp_broker_port = "{{ amqp_broker.port }}"
        self.amqp_broker_vhost = "{{ amqp_broker.vhost }}"
        self.username = "{{ amqp_broker.username }}"
        self.password = "{{ amqp_broker.password }}"
        self.ros_message_type = {{ msgType }}
        self.ros_node_name = self.__class__.__name__
        self._debug = False

    @property
    def debug(self):
        return self._debug

    @debug.setter
    def debug(self, val):
        self._debug = val

    def run(self):
        """Start the Bridge"""
        self._init_ros_subscriber()
        self._init_platform_publisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.001)  # Sleep for 1ms to free cpu resources

    def _init_ros_subscriber(self):
        rospy.init_node(self.ros_node_name)
        rospy.loginfo('ROS Subscriber <{}> ready!'.format(self.ros_topic))
        rospy.Subscriber(self.ros_topic, self.ros_message_type,
                         self._ros_callback)

    def _ros_callback(self, msg):
        try:
            data = ros_msg_to_dict(msg)
            self._publish(data)
        except Exception as exc:
            rospy.lowarn()

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
            raise TypeError('Data should be of type dict')
        self.pub.publish(data)
        rospy.loginfo('Publishing message: {}'.format(data))


if __name__ == '__main__':
    br = {{ conn_name }}()
    br.run()
