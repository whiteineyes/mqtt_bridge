# -*- coding: utf-8 -*-
# 导入标准的库，而非本地库
from __future__ import absolute_import

import inject
import paho.mqtt.client as mqtt
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

# 导入本地库
from .bridge import create_bridge
from .mqtt_client import create_private_path_extractor
from .util import lookup_object

# 创建配置
def create_config(mqtt_client, serializer, deserializer, mqtt_private_path):
    if isinstance(serializer, basestring):
        serializer = lookup_object(serializer)
    if isinstance(deserializer, basestring):
        deserializer = lookup_object(deserializer)
    private_path_extractor = create_private_path_extractor(mqtt_private_path)
    def config(binder):
        binder.bind('serializer', serializer)
        binder.bind('deserializer', deserializer)
        binder.bind(mqtt.Client, mqtt_client)
        binder.bind('mqtt_private_path_extractor', private_path_extractor)
    return config

# 定义
def mqtt_bridge_node():
    # 初始化节点
    rospy.init_node('mqtt_bridge_node')

    # 载入参数
    params = rospy.get_param("~", {})
    mqtt_params = params.pop("mqtt", {})
    conn_params = mqtt_params.pop("connection")
    mqtt_private_path = mqtt_params.pop("private_path", "")
    bridge_params = params.get("bridge", [])

    # 创建 mqtt client
    mqtt_client_factory_name = rospy.get_param(
        "~mqtt_client_factory", ".mqtt_client:default_mqtt_client_factory")
    mqtt_client_factory = lookup_object(mqtt_client_factory_name)
    mqtt_client = mqtt_client_factory(mqtt_params)

    # 加载serializer and deserializer
    serializer = params.get('serializer', 'json:dumps')
    deserializer = params.get('deserializer', 'json:loads')

    # 依赖注入
    config = create_config(
        mqtt_client, serializer, deserializer, mqtt_private_path)
    inject.configure(config)

    # 配置 MQTT broker， 连接到 MQTT broker
    mqtt_client.on_connect = _on_connect
    mqtt_client.on_disconnect = _on_disconnect
    mqtt_client.connect(**conn_params)

    # 配置 bridges
    bridges = []
    for bridge_args in bridge_params:
        bridges.append(create_bridge(**bridge_args))

    # 开启 MQTT loop
    mqtt_client.loop_start()

    # 注册shutdown的回调 ， 开始spin
    rospy.on_shutdown(mqtt_client.disconnect)
    rospy.on_shutdown(mqtt_client.loop_stop)
    rospy.spin()

# 定义_on_connect
def _on_connect(client, userdata, flags, response_code):
    rospy.loginfo('MQTT connected')

# 定义_on_disconnect
def _on_disconnect(client, userdata, response_code):
    rospy.loginfo('MQTT disconnected')

# mqtt_bridge_node可以外部引用
__all__ = ['mqtt_bridge_node']
