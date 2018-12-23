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
        self.amqp_broker_vhost = "{{ amqp_broker.vhost }}"
        self.username = "{{ amqp_broker.username }}"
        self.password = "{{ amqp_broker.password }}"
        self.ros_message_type = {{ bridge.rosMessageType }}
        self.ros_node_name = self.__class__.__name__
        self._debug = False

    @property
    def debug(self):
        return self._debug

    @debug.setter(self, val):
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
        ros_msg = dict_to_ros_msg('ledstrip_hw_interface/RgbLedArray', msg)
        self.pub.publish(ros_msg)

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
    br = {{ bridge.name }}Bridge()
    br.run()
