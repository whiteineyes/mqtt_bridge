# mqtt_bridge

mqtt_bridge provides a functionality to bridge between ROS and MQTT in bidirectional.  
mqtt_bridge 为 ROS 和 MQTT 双向通讯提供了桥接功能


## 1.Principle

`mqtt_bridge` uses ROS message as its protocol. Messages from ROS are serialized by json (or messagepack) for MQTT, and messages from MQTT are deserialized for ROS topic. So MQTT messages should be ROS message compatible. (We use `rosbridge_library.internal.message_conversion` for message conversion.)  

`mqtt_bridge` 使用ROS message 作为它的协议， 将来自ROS的Messages 序列化成 MQTT的json， 将来自MQTT的消息被反序列化成为ROS topic，以达到MQTT messages 和 ROS message 的兼容目的。（使用`rosbridge_library.internal.message_conversion`来进行 message 转换）  

This limitation can be overcome by defining custom bridge class, though.



## 2.Demo

### 2.1 prepare MQTT broker and client  
### 2.1 准备MQTT的broker和客户端  
```
$ sudo apt-get install mosquitto mosquitto-clients  
```

### 2.2 Install python modules  
### 2.2 确保Python版本为2,尽量使用pip2 install  

```bash
$ pip install -r requirements.txt  
```

### 2.3 launch node  

``` bash
$ roslaunch mqtt_bridge demo.launch  
```
#### 会提示报错，根据提示来安装缺少的python包，ros默认python2，尽量安装python2支持的包  
python2 最高只支持inject3.5.4  
ros-`****`-rosbridge-library中，`****`替换成自己的ros版本  
不支持bson，需要卸载，然后安装pymongo  
```bash
$ sudo pip install inject==3.5.4  
$ sudo pip install paho-mqtt  
$ sudo apt install ros-****-rosbridge-library  
$ sudo pip uninstall bson  
$ python -m pip install pymongo  
``` 
#### 安装完上述包后，再次启动launch  
```bash
$ roslaunch mqtt_bridge demo.launch  
```
### 2.4 本地测试效果
开启一个终端，用于查看`/pong`的回应  

```bash 
$ rostopic echo /pong  
``` 
再开启另外一个终端，用于发布消息到`/ping`  
```bash 
$ rostopic pub /ping std_msgs/Bool "data: true"  
```

切换回可以`/pong`的终端，可以看到回应  

```
$ rostopic echo /pong  
data: True  
---  
```
下列示例同上  
Publish "hello" to `/echo`  

```
$ rostopic pub /echo std_msgs/String "data: 'hello'"  
```

and see response to `/back`  

```
$ rostopic echo /back  
data: hello  
---
```

You can also see MQTT messages using `mosquitto_sub`  

```
$ mosquitto_sub -t '#'  
```
## 3. 适配slam算法  
以激光SLAM为例，假设需要发送 激光SLAM发布的点云消息`sensor_msgs.msg:PointCloud2`，`topic`为`/segmented_cloud`    
### 3.1 在 `/config/demo_params.yaml`中，添加所需要中转的消息`bridge`，以下为示例,修改`msg_type`、`topic_from`和`topic_to`：  
```bash
  # 格式：sensor_msgs.msg:PointCloud2   topic:/segmented_cloud
  # ros topic：/segmented_cloud -> MQTT内部序列化 -> segmented_cloud -> 反序列化成ros消息-> 发布到topic：/segmented_cloud_fromMqtt
  - factory: mqtt_bridge.bridge:RosToMqttBridge
    msg_type: sensor_msgs.msg:PointCloud2
    topic_from: /segmented_cloud
    topic_to: segmented_cloud
    
  - factory: mqtt_bridge.bridge:MqttToRosBridge
    msg_type: sensor_msgs.msg:PointCloud2
    topic_from: segmented_cloud
    topic_to: /segmented_cloud_fromMqtt
```
### 3.2 在`package.xml`中添加需要的消息类型，供编译   
```xml
  <exec_depend>sensors_msgs</exec_depend>
```
### 3.3 在 `app.py`、`bridge.py`和`mqtt_client.py` 头部添加消息库  
```bash
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
```
### 3.4 编译  
```catkin_make```
### 3.5 启动launch
```bash
$ roslaunch mqtt_bridge demo.launch  
```
### 3.6 效果举例
在ros中，正常使用其他slam，在点云rivz中，添加一个topic`/segmented_cloud_fromMqtt`



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
