#!/usr/bin/env python2

import rospy

{% set msgPkg = rospub.topic.msgType.split('/')[0] %}
{% set msgType = rospub.topic.msgType.split('/')[1] %}
from {{ msgPkg }}.msg import {{ msgType }}

import amqp_common
from rosconversions import ros_msg_to_dict


class {{ conn_name }}(object):
    """TODO!"""

    def __init__(self, connection=None):
        self.ros_topic = "{{ rospub.topic.uri }}"
        self.amqp_exchange = "{{ amqp_topic.exchange }}"
        self.amqp_topic = "{{ amqp_topic.uri }}"
        self.amqp_topic_namespace = "{{ amqp_broker.namespace }}"
        self.amqp_broker_ip = "{{ amqp_broker.ip }}"
        self.amqp_broker_port = "{{ amqp_broker.port }}"
        self.amqp_broker_vhost = "{{ amqp_broker.vhost }}"
        self.username = "{{ amqp_broker.username }}"
        self.password = "{{ amqp_broker.password }}"
        self.ros_message_type = {{ msgType }}
        self.ros_node_name = self.__class__.__name__
        self._debug = False

        if self.amqp_topic_namespace != '':
            self.amqp_topic = '{}.{}'.format(self.amqp_topic_namespace,
                                             self.amqp_topic)

        if connection:
            self.broker_conn = connection
            return
        self.broker_conn_params = amqp_common.ConnectionParameters(
            host=self.amqp_broker_ip, port=self.amqp_broker_port,
            vhost=self.amqp_broker_vhost)
        self.broker_conn_params.credentials = amqp_common.Credentials(
            self.username, self.password)
        self.broker_conn = amqp_common.SharedConnection(self.broker_conn_params)

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
        rospy.loginfo('Connector [ROS:{} -> AMQP:{}] ready'.format(self.ros_topic, self.amqp_topic))
        while not rospy.is_shutdown():
            self.broker_conn.sleep(0.01)  # Sleep for 1ms to free cpu resources

    def _init_ros_subscriber(self):
        if self.debug:
            log_level = rospy.DEBUG
        else:
            log_level = rospy.INFO
        rospy.init_node(self.ros_node_name, anonymous=True, log_level=log_level)
        rospy.Subscriber(self.ros_topic, self.ros_message_type,
                         self._ros_callback)
        rospy.loginfo('ROS Subscriber <{}> ready!'.format(self.ros_topic))

    def _ros_callback(self, msg):
        try:
            data = ros_msg_to_dict(msg)
        except Exception as exc:
            rospy.logerr('Serialization exception thrown: {}'.format(str(exc)))
            return
        self._publish(data)

    def _init_platform_publisher(self):
        self.broker_pub = amqp_common.PublisherSync(
            self.amqp_topic, connection=self.broker_conn)
        rospy.loginfo('AMQP Publisher <{}> ready!'.format(self.amqp_topic))

    def _publish(self, data):
        if not isinstance(data, dict):
            raise TypeError('Data should be of type dict')
        try:
            self.broker_pub.publish(data)
            rospy.logdebug('Published message: {}'.format(data))
        except Exception as e:
            rospy.logerr('Exception thrown while trying ' +
                         'to publish data to AMQP broker: {}'.format(str(e)))


if __name__ == '__main__':
    br = {{ conn_name }}()
    br.run()
