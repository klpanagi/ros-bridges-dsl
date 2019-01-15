#!/usr/bin/env python

import sys
from os.path import dirname, join, exists, isdir
from os import mkdir, chmod

import shutil

import jinja2
from textx import metamodel_from_file

from _logging import create_logger


class ROSPackageCfg(object):
    __slots__ = ['name', 'description', 'version', 'rosdeps']

    def __init__(self,
                 name='platform_bridges',
                 description='ROS<->AMQP bridges',
                 version='0.1.0',
                 rosdeps=['rospy', 'std_msgs', 'sensor_msgs']):
        self.name = name
        self.description = description
        self.version = version
        self.rosdeps = rosdeps


class Generator(object):
    def __init__(self):
        self.this_dir = dirname(__file__)
        self.gen_dir = join(self.this_dir, 'gen')
        self.tpl_dir = join(self.this_dir, 'templates')
        self.logger = create_logger(self.__class__.__name__)

    def generate(self, model):
        """Abstract. Implement."""
        raise NotImplementedError()

    def _mkdir(self, destdir):
        # Create output folder
        if not exists(destdir):
            mkdir(destdir)

    def _cpdir(self, src, dest):
        if exists(dest) and isdir(dest):
            shutil.rmtree(dest)
        shutil.copytree(src, dest)

    def _init_tpl_engine(self):
        self.jinja_env = jinja2.Environment(
            loader=jinja2.FileSystemLoader(self.this_dir),
            trim_blocks=True,
            lstrip_blocks=True)

    def _create_ros_py_pkg(self, package):
        ros_py_pkg_tpl = join(self.tpl_dir, 'ros_py_pkg_tpl')
        pkg_root = join(self.gen_dir, package.name)
        self._mkdir(pkg_root)
        self._mkdir(join(pkg_root, 'src'))
        self._mkdir(join(pkg_root, 'launch'))
        self._mkdir(join(pkg_root, 'scripts'))
        self._mkdir(join(pkg_root, 'cfg'))

        cmake_tpl = join(ros_py_pkg_tpl, 'CMakeLists.tpl.txt')
        pkg_xml_tpl = join(ros_py_pkg_tpl, 'package.tpl.xml')
        setup_tpl = join(ros_py_pkg_tpl, 'setup.tpl.py')

        tpl = self.jinja_env.get_template(cmake_tpl)
        gen_path = join(pkg_root, 'CMakeLists.txt')
        with open(gen_path, 'w') as f:
            f.write(tpl.render(package=package))

        tpl = self.jinja_env.get_template(pkg_xml_tpl)
        gen_path = join(pkg_root, 'package.xml')
        with open(gen_path, 'w') as f:
            f.write(tpl.render(package=package))

        tpl = self.jinja_env.get_template(setup_tpl)
        gen_path = join(pkg_root, 'setup.py')
        with open(gen_path, 'w') as f:
            f.write(tpl.render(package=package))

        return pkg_root

    def _gen_launch_file(self, package, amqppubs, launcher_name='launcher'):
        ros_py_pkg_tpl = join(self.tpl_dir, 'ros_py_pkg_tpl')
        pkg_root = join(self.gen_dir, package.name)
        launch_tpl = join(ros_py_pkg_tpl, 'launcher.tpl.launch')
        tpl = self.jinja_env.get_template(launch_tpl)
        gen_path = join(pkg_root, 'launch', '{}.launch'.format(launcher_name))
        with open(gen_path, 'w') as f:
            f.write(tpl.render(package=package, amqppubs=amqppubs))


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

        bridge_tpl = join(self.tpl_dir, 'ros_amqp_pub_bridge.tpl.py')
        tpl = self.jinja_env.get_template(bridge_tpl)

        for conn in model.rosPubConnectors:
            ros_pub = conn.rosPub
            amqp_topic = conn.amqpTopic
            # For each amqppub generate file
            name = ros_pub.topic.uri.replace('/', '_')
            if name[0] == '_':
                # Remove first dot if exists
                name = name[1:]
            gen_path = join(pkg_root, 'scripts', '{}_bridge.py'.format(name))
            # Append the name of the ROS node
            ros_node_name = '{}_bridge'.format(name)
            with open(gen_path, 'w') as f:
                f.write(
                    tpl.render(
                        amqp_broker=broker,
                        rospub=ros_pub,
                        amqp_topic=amqp_topic,
                        conn_name=conn.name.replace('_', '')))
            # Give execution permissions to the generated python file
            chmod(gen_path, 509)

            self._log_gen_pub(conn, gen_path)

        bridge_tpl = join(self.tpl_dir, 'ros_amqp_sub_bridge.tpl.py')
        tpl = self.jinja_env.get_template(bridge_tpl)
        for conn in model.rosSubConnectors:
            ros_sub = conn.rosSub
            amqp_topic = conn.amqpTopic
            # For each amqppub generate file
            name = ros_sub.topic.uri.replace('/', '_')
            if name[0] == '_':
                # Remove first dot if exists
                name = name[1:]
            gen_path = join(pkg_root, 'scripts', '{}_bridge.py'.format(name))
            # Append the name of the ROS node
            ros_node_name = '{}_bridge'.format(name)
            with open(gen_path, 'w') as f:
                f.write(
                    tpl.render(
                        amqp_broker=broker,
                        rossub=ros_sub,
                        amqp_topic=amqp_topic,
                        conn_name=conn.name.replace('_', '')))
            # Give execution permissions to the generated python file
            chmod(gen_path, 509)

            self._log_gen_sub(conn, gen_path)

        # Generate launch file
        #  self._gen_launch_file(pkg_cfg, model.rosPubConnectors)

    def _log_gen_pub(self, conn, pkg_root):
        ros_pub = conn.rosPub
        amqp_topic = conn.amqpTopic
        self.logger.debug(
            '[*] Generated ROS Publisher -> AMQP connector:\n' +
            '- Destination: {}\n'.format(pkg_root) + '- Topic Map: {} -> {}.{}'
            .format(ros_pub.topic.uri, amqp_topic.namespace, amqp_topic.uri))

    def _log_gen_sub(self, conn, pkg_root):
        ros_sub = conn.rosSub
        amqp_topic = conn.amqpTopic
        self.logger.debug(
            '[*] Generated ROS Publisher -> AMQP connector:\n' +
            '- Destination: {}\n'.format(pkg_root) + '- Topic Map: {} -> {}.{}'
            .format(ros_sub.topic.uri, amqp_topic.namespace, amqp_topic.uri))


if __name__ == '__main__':
    this_folder = dirname(__file__)
    if len(sys.argv) != 2:
        print("[*] - Usage: python {} <model>".format(sys.argv[0]))
    else:
        meta_model = metamodel_from_file(join(this_folder, 'metamodel2.tx'))
        model = meta_model.model_from_file(sys.argv[1])
        print(model)
        gen = RosAMQPBridgesGen()
        gen.generate(model)
