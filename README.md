# mqtt_bridge

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.
mqtt_bridge 为 ROS 和 MQTT 双向通讯提供了桥接功能


## Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)

This limitation can be overcome by defining custom bridge class, though.

`mqtt_bridge` 使用ROS message 作为它的协议， 将来自ROS的Messages 序列化成 MQTT的json， 将来自MQTT的消息被反序列化成为ROS topic. 因此MQTT messages 和 ROS message 可以做到兼容。（使用`rosbridge_library.internal.message_conversion`来进行 message 转换）

## Demo

### prepare MQTT broker and client

```
$ sudo apt-get install mosquitto mosquitto-clients
```

### Install python modules
### 确保Python版本为2,尽量使用pip2 install

```bash
$ pip install -r requirements.txt
```

### launch node
### 会提示报错，根据提示来安装缺少的python包，ros默认python2，尽量安装python2支持的包
#### 1 sudo pip install inject==3.5.4
#### 2 sudo pip install paho-mqtt
#### 3 sudo apt install ros-melodic-rosbridge-library
#### 4.1 sudo pip uninstall bson
#### 4.2 python -m pip install pymongo

``` bash
$ roslaunch mqtt_bridge demo.launch
```

Publish to `/ping`,

```
$ rostopic pub /ping std_msgs/Bool "data: true"
```

and see response to `/pong`.

```
$ rostopic echo /pong
data: True
---
```

Publish "hello" to `/echo`,

```
$ rostopic pub /echo std_msgs/String "data: 'hello'"
```

and see response to `/back`.

```
$ rostopic echo /back
data: hello
---
```

You can also see MQTT messages using `mosquitto_sub`

```
$ mosquitto_sub -t '#'
```

## Usage

parameter file (config.yaml):

``` yaml
mqtt:
  client:
    protocol: 4      # MQTTv311
  connection:
    host: localhost
    port: 1883
    keepalive: 60
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

launch file:

``` xml
<launch>
  <node name="mqtt_bridge" pkg="mqtt_bridge" type="mqtt_bridge_node.py" output="screen">
    <rosparam file="/path/to/config.yaml" command="load" />
  </node>
</launch>
```


## Configuration

### mqtt

Parameters under `mqtt` section are used for creating paho's `mqtt.Client` and its configuration.

#### subsections

* `client`: used for `mqtt.Client` constructor
* `tls`: used for tls configuration
* `account`: used for username and password configuration
* `message`: used for MQTT message configuration
* `userdata`: used for MQTT userdata configuration
* `will`: used for MQTT's will configuration

See `mqtt_bridge.mqtt_client` for detail.

### mqtt private path

If `mqtt/private_path` parameter is set, leading `~/` in MQTT topic path will be replaced by this value. For example, if `mqtt/pivate_path` is set as "device/001", MQTT path "~/value" will be converted to "device/001/value".

### serializer and deserializer

`mqtt_bridge` uses `json` as a serializer in default. But you can also configure other serializers. For example, if you want to use messagepack for serialization, add following configuration.

``` yaml
serializer: msgpack:dumps
deserializer: msgpack:loads
```

### bridges

You can list ROS <--> MQTT tranfer specifications in following format.

``` yaml
bridge:
  # ping pong
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: std_msgs.msg:Bool
    topic_from: /ping
    topic_to: ping
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: std_msgs.msg:Bool
    topic_from: ping
    topic_to: /pong
```

* `factory`: bridge class for transfering message from ROS to MQTT, and vise versa.
* `msg_type`: ROS Message type transfering through the bridge.
* `topic_from`: topic incoming from (ROS or MQTT)
* `topic_to`: topic outgoing to (ROS or MQTT)

Also, you can create custom bridge class by inheriting `mqtt_brige.bridge.Bridge`.


## License

This software is released under the MIT License, see LICENSE.txt.
