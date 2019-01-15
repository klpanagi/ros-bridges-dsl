#!/usr/bin/env python2

import rospy

{% set msgPkg = rossub.topic.msgType.split('/')[0] %}
{% set msgType = rossub.topic.msgType.split('/')[1] %}
from {{ msgPkg }}.msg import {{ msgType }}

import amqp_common
from rosconversions import dict_to_ros_msg


class {{ conn_name }}Bridge(object):
    """TODO!"""

    def __init__(self):
        self.ros_topic = "{{ rossub.topic.uri }}"
        self.amqp_exchange = "{{ amqp_topic.exchange }}"
        self.amqp_topic = "{{ amqp_topic.uri }}"
        self.amqp_topic_namespace = "{{ amqp_topic.namespace }}"
        self.amqp_broker_ip = "{{ amqp_broker.ip }}"
        self.amqp_broker_port = "{{ amqp_broker.port }}"
        self.amqp_broker_vhost = "{{ amqp_topic.vhost }}"
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
        self._init_platform_subscriber()
        self._init_ros_publisher()
        while not rospy.is_shutdown():
            rospy.sleep(0.001)

    def _init_ros_publisher(self):
        rospy.init_node(self.ros_node_name)
        self.pub = rospy.Publisher(self.ros_topic, self.ros_message_type,
                                   queue_size=10)
        rospy.loginfo('ROS Publisher <{}> ready!'.format(self.ros_topic))

    def _callback(self, msg, meta):
        try:
            ros_msg = dict_to_ros_msg('{{ rossub.topic.msgType }}', msg)
            self.pub.publish(ros_msg)
        except Exception as exc:
            rospy.loginfo('Could not convert input message [{}] to {{ rossub.topic.msgType }}'.format(msg))

    def _init_platform_subscriber(self):
        self.sub = amqp_common.SubscriberSync(
            self.amqp_topic, on_message=self._callback,
            connection_params=amqp_common.ConnectionParameters(
                host=self.amqp_broker_ip, port=self.amqp_broker_port,
                vhost=self.amqp_broker_vhost),
            queue_size=10,
            creds=amqp_common.Credentials(self.username, self.password),
            debug=self.debug
        )
        rospy.loginfo('AMQP Subscriber <{}> ready!'.format(self.ros_topic))
        self.sub.run_threaded()


if __name__ == '__main__':
    br = {{ conn_name }}Bridge()
    br.run()
