# MQTT数据客户端使用说明

小智AI支持通过MQTT协议将设备采集的数据发送到MQTT服务器，可作为物联网数据采集节点使用。

## 功能特点

- 独立于主要通信协议（WebSocket）运行，不互相干扰
- 支持自定义MQTT服务器地址、端口、用户名、密码
- 支持自定义发布主题和发布间隔
- 采集并发送设备信息、系统信息、电池状态和温度等数据
- 支持通过MCP协议远程配置和控制

## 配置方法

### 方法一：通过menuconfig配置（编译时）

1. 运行 `idf.py menuconfig`
2. 进入 `Xiaozhi Assistant` 菜单
3. 启用 `Enable MQTT Data Client`
4. 配置默认的MQTT服务器地址、端口和主题
5. 保存配置并编译固件

### 方法二：通过MCP协议配置（运行时）

使用MCP协议可以在设备运行时配置MQTT客户端。MQTT客户端作为一个Thing对象注册到系统中，可以通过MCP协议进行控制。

#### 属性

- `enabled`: 是否启用MQTT客户端
- `broker`: MQTT服务器地址
- `port`: MQTT服务器端口
- `topic`: 发布主题
- `interval`: 数据发布间隔（毫秒）

#### 方法

- `start`: 启动MQTT客户端
  - 参数:
    - `broker`: MQTT服务器地址（必需）
    - `port`: MQTT服务器端口（可选，默认1883）
    - `username`: 用户名（可选）
    - `password`: 密码（可选）
    - `topic`: 发布主题（可选，默认"xiaozhi/data"）
    - `interval`: 发布间隔（可选，默认10000毫秒）

- `stop`: 停止MQTT客户端

## 数据格式

MQTT客户端发布的数据采用JSON格式，包含以下信息：

```json
{
  "device_id": "xiaozhi_xxxxxxxxxxxx",
  "mac": "xx:xx:xx:xx:xx:xx",
  "chip": "esp32s3",
  "system": {
    "free_heap": 123456,
    "min_free_heap": 100000,
    "flash_size": 8388608
  },
  "battery": {
    "level": 85,
    "charging": false,
    "discharging": true
  },
  "temperature": 36.5
}
```

## 使用示例

### 通过语音控制（MCP协议）

可以通过语音指令控制MQTT客户端，例如：

- "小智，启动MQTT客户端，连接到192.168.1.100"
- "小智，停止MQTT客户端"
- "小智，设置MQTT发布间隔为5秒"

### 通过代码控制

```cpp
// 在应用程序中获取Thing Manager
auto& thing_manager = iot::ThingManager::GetInstance();

// 创建MQTT客户端Thing
auto mqtt_client = iot::CreateThing("MqttClient");
if (mqtt_client) {
    thing_manager.AddThing(mqtt_client);
}

// 通过JSON命令控制
cJSON* command = cJSON_CreateObject();
cJSON_AddStringToObject(command, "name", "MqttClient");
cJSON_AddStringToObject(command, "method", "start");

cJSON* parameters = cJSON_CreateObject();
cJSON_AddStringToObject(parameters, "broker", "192.168.1.100");
cJSON_AddNumberToObject(parameters, "port", 1883);
cJSON_AddStringToObject(parameters, "username", "user");
cJSON_AddStringToObject(parameters, "password", "pass");
cJSON_AddStringToObject(parameters, "topic", "xiaozhi/data");
cJSON_AddNumberToObject(parameters, "interval", 5000);

cJSON_AddItemToObject(command, "parameters", parameters);

// 执行命令
thing_manager.Invoke(command);

// 清理
cJSON_Delete(command);
```

## 注意事项

1. MQTT客户端和主WebSocket通信使用不同的任务和网络资源，不会相互干扰
2. 发布间隔不宜设置过短，建议至少5秒以上，避免占用过多网络资源
3. 如果设备处于低电量状态，MQTT客户端会自动降低发布频率或暂停发布
4. 数据发送失败会自动重试，但不会无限重试，避免消耗过多资源
5. 重启设备后，如果之前已配置并启用MQTT客户端，会自动重新连接并继续发送数据 