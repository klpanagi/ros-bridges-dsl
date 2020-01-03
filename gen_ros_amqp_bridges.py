#!/usr/bin/env python

import sys
from os.path import dirname, join, exists, isdir
from os import mkdir, chmod

import shutil

import yaml

import jinja2

from _logging import create_logger

TEMPLATE_DIR_PATH = join(dirname(__file__), 'templates')
GEN_DIR_PATH = join(TEMPLATE_DIR_PATH, '..', 'gen')


class ROSPackageCfg(object):
    __slots__ = ['name', 'description', 'version', 'rosdeps']

    def __init__(self,
                 name='platform_bridges',
                 description='ROS-AMQP bridges',
                 version='0.1.0',
                 rosdeps=['rospy', 'std_msgs', 'sensor_msgs']):
        self.name = name
        self.description = description
        self.version = version
        self.rosdeps = rosdeps

def _mkdir(self, destdir):
    # Create output folder
    if not exists(destdir):
        mkdir(destdir)


def _cpdir(self, src, dest):
    if exists(dest) and isdir(dest):
        shutil.rmtree(dest)
    shutil.copytree(src, dest)


class Generator(object):
    def __init__(self):
        self.this_dir = dirname(__file__)
        self.gen_dir = join(self.this_dir, 'gen')
        self.tpl_dir = join(self.this_dir, 'templates')
        self.logger = create_logger(self.__class__.__name__)

    def generate(self, model):
        """Abstract. Implement."""
        raise NotImplementedError()

    def _init_tpl_engine(self):
        self.jinja_env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(self.this_dir),
            trim_blocks=True,
            lstrip_blocks=True)

    def _gen_launch_file(self, package, bridges, launcher_name='launcher'):
        ros_py_pkg_tpl = join(self.tpl_dir, 'ros_py_pkg_tpl')
        pkg_root = join(self.gen_dir, package.name)
        launch_tpl = join(ros_py_pkg_tpl, 'launcher.tpl.launch')
        tpl = self.jinja_env.get_template(launch_tpl)
        gen_path = join(pkg_root, 'launch', '{}.launch'.format(launcher_name))
        with open(gen_path, 'w') as f:
            f.write(tpl.render(package=package, bridges=bridges))


class RosAMQPBridgesGen(Generator):
    def __init__(self):
        Generator.__init__(self)

    def generate(self, model):
        broker = model.amqpBroker
        topics = broker.topics
        ros_sys = model.rosSystem

        self.logger.debug('[*] - AMQP Broker: {}:{}'.format(
            broker.ip, broker.port))
        self._mkdir(self.gen_dir)

        self._init_tpl_engine()

        pkg_cfg = ROSPackageCfg()

        pkg_root = self._create_ros_py_pkg(pkg_cfg)
        self._gen_pubConnectors(model, pkg_root)
        self._gen_subConnectors(model, pkg_root)
        self._gen_rpcConnectors(model, pkg_root)
        br = model.rosServiceConnectors + model.rosPubConnectors + \
            model.rosSubConnectors
        self._gen_launch_file(pkg_cfg, br)

    def _gen_rpcConnectors(self, model, dest_pkg):
        bridge_tpl = join(self.tpl_dir, 'ros_amqp_rpc_bridge.tpl.py')
        tpl = self.jinja_env.get_template(bridge_tpl)

        for conn in model.rosServiceConnectors:
            ros_service = conn.rosService
            amqp_rpc = conn.amqpRPC
            # For each amqppub generate file
            node_exec_name = conn.name.lower()
            gen_path = join(dest_pkg, 'scripts', '{}.py'.format(
                node_exec_name))
            # Append the name of the ROS node
            with open(gen_path, 'w') as f:
                f.write(
                    tpl.render(
                        amqp_broker=model.amqpBroker,
                        ros_service=ros_service,
                        amqp_rpc=amqp_rpc,
                        conn_name=conn.name.replace('_', '')))
            # Give execution permissions to the generated python file
            chmod(gen_path, 509)
            self._log_gen_rpc(conn, gen_path)

    def _gen_pubConnectors(self, model, dest_pkg):
        bridge_tpl = join(self.tpl_dir, 'ros_amqp_pub_bridge.tpl.py')
        tpl = self.jinja_env.get_template(bridge_tpl)

        for conn in model.rosPubConnectors:
            # For each amqppub generate file
            ros_pub = conn.rosPub
            amqp_topic = conn.amqpTopic
            node_exec_name = conn.name.lower()
            gen_path = join(dest_pkg, 'scripts', '{}.py'.format(
                node_exec_name))
            # Append the name of the ROS node
            with open(gen_path, 'w') as f:
                f.write(
                    tpl.render(
                        amqp_broker=model.amqpBroker,
                        rospub=ros_pub,
                        amqp_topic=amqp_topic,
                        conn_name=conn.name.replace('_', '')))
            # Give execution permissions to the generated python file
            chmod(gen_path, 509)

            self._log_gen_pub(conn, gen_path)

    def _gen_subConnectors(self, model, dest_pkg):
        bridge_tpl = join(self.tpl_dir, 'ros_amqp_sub_bridge.tpl.py')
        tpl = self.jinja_env.get_template(bridge_tpl)

        for conn in model.rosSubConnectors:
            # For each amqpsub generate file
            ros_sub = conn.rosSub
            amqp_topic = conn.amqpTopic
            node_exec_name = conn.name.lower()
            # Append the name of the ROS node
            gen_path = join(dest_pkg, 'scripts', '{}.py'.format(
                node_exec_name))
            with open(gen_path, 'w') as f:
                f.write(
                    tpl.render(
                        amqp_broker=model.amqpBroker,
                        rossub=ros_sub,
                        amqp_topic=amqp_topic,
                        conn_name=conn.name.replace('_', '')))
            # Give execution permissions to the generated python file
            chmod(gen_path, 509)

            self._log_gen_sub(conn, gen_path)

    def _log_gen_pub(self, conn, pkg_root):
        ros_pub = conn.rosPub
        amqp_topic = conn.amqpTopic
        self.logger.debug(
            '[*] Generated Connector: [ROS-Topic:{} -> AMQP-Topic:{}]\n'.format(
                ros_pub.topic.uri, amqp_topic.uri))

    def _log_gen_rpc(self, conn, pkg_root):
        ros_srv = conn.rosService
        amqp_rpc = conn.amqpRPC
        self.logger.debug(
            '[*] Generated Connector: [AMQP RPC:{} -> ROS Service:{}]\n'.format(
                ros_srv.uri, amqp_rpc.uri))

    def _log_gen_sub(self, conn, pkg_root):
        ros_sub = conn.rosSub
        amqp_topic = conn.amqpTopic
        self.logger.debug(
            '[*] Generated Connector: [AMQP-Topic:{} -> ROS-Topic:{}]\n'.format(
                amqp_topic.uri, ros_sub.topic.uri))


def _create_ros_py_pkg(self, package):
    ros_py_pkg_tpl = join(TEMPLATE_DIR_PATH, 'ros_py_pkg_tpl')
    this_dir = dirname(__file__)
    pkg_root = join(GEN_DIR_PATH, package.name)
    _mkdir(pkg_root)
    _mkdir(join(pkg_root, 'src'))
    _mkdir(join(pkg_root, 'launch'))
    _mkdir(join(pkg_root, 'scripts'))
    _mkdir(join(pkg_root, 'cfg'))

    cmake_tpl = join(ros_py_pkg_tpl, 'CMakeLists.tpl.txt')
    pkg_xml_tpl = join(ros_py_pkg_tpl, 'package.tpl.xml')
    setup_tpl = join(ros_py_pkg_tpl, 'setup.tpl.py')

    jinja_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(this_dir),
        trim_blocks=True,
        lstrip_blocks=True)

    tpl = jinja_env.get_template(cmake_tpl)
    gen_path = join(pkg_root, 'CMakeLists.txt')
    with open(gen_path, 'w') as f:
        f.write(tpl.render(package=package))

    tpl = jinja_env.get_template(pkg_xml_tpl)
    gen_path = join(pkg_root, 'package.xml')
    with open(gen_path, 'w') as f:
        f.write(tpl.render(package=package))

    tpl = jinja_env.get_template(setup_tpl)
    gen_path = join(pkg_root, 'setup.py')
    with open(gen_path, 'w') as f:
        f.write(tpl.render(package=package))

    return pkg_root


def parse_yaml(fpath):
    with open(fpath, 'r') as f:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        model = yaml.load(f, Loader=yaml.FullLoader)
        return model


def generate(model):
    jinja_env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(dirname(__file__)),
        trim_blocks=True,
        lstrip_blocks=True)

    connectors = model['connector']
    broker = model['connector'][0]  # Only 1 broker is currently supported

    for k in connectors:
        print(k)


if __name__ == '__main__':
    this_folder = dirname(__file__)
    if len(sys.argv) != 2:
        print('[*] - Usage: python {} <model>'.format(sys.argv[0]))
    else:
        fpath = sys.argv[1]
        model = parse_yaml(fpath)
        generate(model)
