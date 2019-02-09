#!/usr/bin/env python2

import amqp_common

import rospy

{% set srvPkg = ros_service.srvType.split('/')[0] %}
{% set srvType = ros_service.srvType.split('/')[1] %}
from {{ srvPkg }}.srv import {{ srvType }}, {{ srvType }}Request, {{ srvType }}Response

from rosconversions import ros_srv_resp_to_dict, dict_to_ros_srv_request


class {{ conn_name }}(object):

    def __init__(self, connection=None):
        self.ros_service_uri = '{{ ros_service.uri }}'
        self.amqp_exchange = '{{ amqp_rpc.exchange }}'
        self.amqp_rpc_name = '{{ amqp_rpc.uri }}'
        self.amqp_rpc_namespace = '{{ amqp_broker.namespace }}'
        self.amqp_broker_ip = '{{ amqp_broker.ip }}'
        self.amqp_broker_port = '{{ amqp_broker.port }}'
        self.amqp_broker_vhost = '{{ amqp_broker.vhost }}'
        self.username = '{{ amqp_broker.username }}'
        self.password = '{{ amqp_broker.password }}'

        if self.amqp_rpc_namespace:
            self.rpc_name = '{}.{}'.format(self.amqp_rpc_namespace,
                                           self.amqp_rpc_name)
        else:
            self.rpc_name = self.amqp_rpc_name
        self.ros_service_type = {{ srvType }}
        self.ros_srv_type_str = '{{ ros_service.srvType }}'
        self.ros_node_name = self.__class__.__name__

        if connection:
            self.broker_conn = connection
            return
        self.conn_params = amqp_common.ConnectionParameters(
            vhost=self.amqp_broker_vhost,
            host=self.amqp_broker_ip,
            port=self.amqp_broker_port)
        self.conn_params.credentials = amqp_common.Credentials(
            self.username, self.password)
        self.broker_conn = amqp_common.SharedConnection(self.conn_params)

    def _init_rpc_server(self):
        self.rpc_server = amqp_common.RpcServer(
            self.rpc_name, on_request=self._rpc_callback,
            connection=self.broker_conn)

    def _init_ros_service(self):
        rospy.init_node(self.ros_node_name, anonymous=True)
        rospy.loginfo('Waiting for ROS Service {} ...'.format(self.ros_service_uri))
        rospy.wait_for_service(self.ros_service_uri)
        self.ros_srv = rospy.ServiceProxy(
            self.ros_service_uri, self.ros_service_type)
        rospy.loginfo('ROS Service Client ready')

    def _rpc_callback(self, msg, meta):
        """."""
        try:
            srv_req = dict_to_ros_srv_request(self.ros_srv_type_str, msg)
            resp = self.ros_srv(srv_req)
        except rospy.ServiceException as exc:
            print('ROS Service call failed: {}'.format(exc))
            resp = {{ srvType }}Response()
        data = ros_srv_resp_to_dict(resp)
        return data

    def run(self, asynchronous=False):
        """Start the bridge."""
        self._init_rpc_server()
        self._init_ros_service()
        self.rpc_server.run_threaded()
        if asynchronous:
            return
        while not rospy.is_shutdown():
            self.broker_conn.sleep(0.01)

if __name__ == "__main__":
    br = {{ conn_name }}()
    br.run()
