#!/usr/bin/env python2

import amqp_common

import rospy

{% set srvPkg = ros_service.srvType.split('/')[0] %}
{% set srvType = ros_service.srvType.split('/')[1] %}
from {{ srvPkg }}.srv import {{ srvType }}

from rosconversions import ros_srv_resp_to_dict


class {{ conn_name }}Bridge(object):

    def __init__(self):
        self.ros_service_uri = "{{ ros_service.uri }}"
        self.amqp_exchange = "{{ amqp_rpc.exchange }}"
        self.amqp_rpc_name = "{{ amqp_rpc.uri }}"
        self.amqp_topic_namespace = "{{ amqp_rpc.namespace }}"
        self.amqp_broker_ip = "{{ amqp_broker.ip }}"
        self.amqp_broker_port = "{{ amqp_broker.port }}"
        self.amqp_broker_vhost = "{{ amqp_rpc.vhost }}"
        self.username = "{{ amqp_broker.username }}"
        self.password = "{{ amqp_broker.password }}"

        self.rpc_name = self.amqp_namespace + \
            self.ros_service_uri.replace('/', '.')
        self.ros_service_type = {{ srvType }}
        self.ros_node_name = self.__class__.__name__

    def _init_rpc_server(self):
        self.rpc_server = amqp_common.RpcServer(
            self.amqp_rpc_name, on_request=self._rpc_callback,
            connection_params=amqp_common.ConnectionParameters(
                host=self.amqp_broker_ip,
                port=self.amqp_broker_port),
            creds=amqp_common.Credentials(self.username,
                                          self.password))

    def _init_ros_service(self):
        rospy.wait_for_service(self.ros_service_uri)
        self.ros_srv = rospy.ServiceProxy(
            self.ros_service_uri, self.ros_service_type)

    def _go_to_next_node(self):
        try:
            resp = self.ros_srv()
            return resp
        except rospy.ServiceException as exc:
            print('Service call failed: {}'.format(exc))

    def _rpc_callback(self):
        """."""
        resp = self._go_to_next_node()
        data = ros_srv_resp_to_dict(resp)
        return data

    def run(self):
        """Start the bridge."""
        self._init_rpc_server()
        self._init_ros_service()
        self.rpc_server.run()


if __name__ == "__main__":
    br = GoToNextNodeBridge()
    br.run()
