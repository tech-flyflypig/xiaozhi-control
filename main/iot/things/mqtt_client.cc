#include "iot/thing.h"
#include "iot/thing_manager.h"
#include "board.h"
#include "application.h"
#include "system_info.h"
#include "settings.h"
#include "sdkconfig.h"

#include <esp_log.h>
#include <mqtt.h>
#include <cJSON.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <inttypes.h>

#define TAG "MqttClient"
#define MQTT_DATA_TASK_STACK_SIZE 4096
#define MQTT_DATA_TASK_PRIORITY 5
#define MQTT_DATA_PUBLISH_INTERVAL_MS 10000 // 10秒发送一次数据

namespace iot
{

    class MqttClient : public Thing
    {
    private:
        Mqtt *mqtt_client_ = nullptr;
        TaskHandle_t mqtt_task_handle_ = nullptr;
        EventGroupHandle_t event_group_ = nullptr;
        bool enabled_ = false;
        std::string broker_ = "";
        int port_ = 1883;
        std::string client_id_ = "";
        std::string username_ = "";
        std::string password_ = "";
        std::string topic_ = "";
        int publish_interval_ms_ = MQTT_DATA_PUBLISH_INTERVAL_MS;

        // 报警阈值
        int hcho_threshold_ = 50;   // 甲醛阈值 (默认50)
        int temp_threshold_ = 3000; // 温度阈值 (默认30.00度)
        int humi_threshold_ = 8000; // 湿度阈值 (默认80.00%)

        // 定义事件位
        static const int MQTT_CONNECTED_BIT = BIT0;
        static const int MQTT_DISCONNECTED_BIT = BIT1;
        static const int MQTT_STOP_BIT = BIT2;

    public:
        MqttClient() : Thing("MqttClient", "MQTT客户端，用于发送设备数据")
        {
            event_group_ = xEventGroupCreate();

            // 定义属性
            properties_.AddBooleanProperty("enabled", "是否启用MQTT客户端", [this]() -> bool
                                           { return enabled_; });
            properties_.AddStringProperty("broker", "MQTT服务器地址", [this]() -> std::string
                                          { return broker_; });
            properties_.AddNumberProperty("port", "MQTT服务器端口", [this]() -> int
                                          { return port_; });
            properties_.AddStringProperty("topic", "MQTT发布主题", [this]() -> std::string
                                          { return topic_; });
            properties_.AddNumberProperty("interval", "数据发布间隔(毫秒)", [this]() -> int
                                          { return publish_interval_ms_; });

            // 定义方法
            ParameterList start_params;
            start_params.AddParameter(Parameter("broker", "MQTT服务器地址", kValueTypeString));
            start_params.AddParameter(Parameter("port", "MQTT服务器端口", kValueTypeNumber, false));
            start_params.AddParameter(Parameter("username", "用户名", kValueTypeString, false));
            start_params.AddParameter(Parameter("password", "密码", kValueTypeString, false));
            start_params.AddParameter(Parameter("topic", "发布主题", kValueTypeString, false));
            start_params.AddParameter(Parameter("interval", "发布间隔(毫秒)", kValueTypeNumber, false));

            methods_.AddMethod("start", "启动MQTT客户端", start_params, [this](const ParameterList &params)
                               {
            broker_ = params["broker"].string();
            
            // 设置可选参数
            try {
                port_ = params["port"].number();
            } catch (const std::runtime_error&) {
                // 使用默认端口
            }
            
            try {
                username_ = params["username"].string();
            } catch (const std::runtime_error&) {
                // 无用户名
            }
            
            try {
                password_ = params["password"].string();
            } catch (const std::runtime_error&) {
                // 无密码
            }
            
            try {
                topic_ = params["topic"].string();
            } catch (const std::runtime_error&) {
                // 使用默认主题
            }
            
            try {
                publish_interval_ms_ = params["interval"].number();
            } catch (const std::runtime_error&) {
                // 使用默认间隔
            }
            
            Start(); });

            ParameterList stop_params;
            methods_.AddMethod("stop", "停止MQTT客户端", stop_params, [this](const ParameterList &params)
                               { Stop(); });

            // 添加阈值相关方法
            methods_.AddMethod("get_thresholds", "获取报警阈值", ParameterList(), [this](const ParameterList &parameters)
                               {
                                   // 返回当前阈值设置
                               });

            methods_.AddMethod("set_threshold", "设置报警阈值", ParameterList({Parameter("type", "阈值类型(hcho/temp/humi)", kValueTypeString, true), Parameter("value", "阈值数值", kValueTypeNumber, true)}), [this](const ParameterList &parameters)
                               {
            std::string type = parameters["type"].string();
            int value = parameters["value"].number();

            if (type == "hcho") {
                hcho_threshold_ = value;
            } else if (type == "temp") {
                temp_threshold_ = value;
            } else if (type == "humi") {
                humi_threshold_ = value;
            }

            SaveThresholds(); });

            // 从设置中加载配置
            LoadConfig();
        }

        ~MqttClient()
        {
            Stop();
            if (event_group_)
            {
                vEventGroupDelete(event_group_);
            }
        }

    private:
        void LoadConfig()
        {
            Settings settings("data_mqtt", false);
            // Kconfig配置是权威配置，如果NVS配置不一致则更新NVS
            bool need_save_config = false;

            // Broker配置
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_BROKER)
            broker_ = CONFIG_MQTT_DATA_CLIENT_BROKER; // 使用Kconfig配置

            // 检查NVS中的配置是否与Kconfig一致
            std::string nvs_broker = settings.GetString("broker");
            if (nvs_broker != broker_)
            {
                need_save_config = true;
            }
#else
            // 没有Kconfig配置时，从NVS读取
            broker_ = settings.GetString("broker");
#endif

            // 端口配置
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_PORT)
            port_ = CONFIG_MQTT_DATA_CLIENT_PORT; // 使用Kconfig配置

            // 检查NVS中的配置是否与Kconfig一致
            int nvs_port = settings.GetInt("port", 0);
            if (nvs_port != port_)
            {
                need_save_config = true;
            }
#else
            // 没有Kconfig配置时，从NVS读取或使用默认值
            port_ = settings.GetInt("port", 1883);
#endif
            client_id_ = settings.GetString("client_id");
            if (client_id_.empty())
            {
                client_id_ = "xiaozhi_" + SystemInfo::GetMacAddress();
                client_id_.erase(std::remove(client_id_.begin(), client_id_.end(), ':'), client_id_.end());
            }
            username_ = settings.GetString("username");
            password_ = settings.GetString("password");
            // Topic配置
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_TOPIC)
            topic_ = CONFIG_MQTT_DATA_CLIENT_TOPIC; // 使用Kconfig配置

            // 检查NVS中的配置是否与Kconfig一致
            std::string nvs_topic = settings.GetString("topic");
            if (nvs_topic != topic_)
            {
                need_save_config = true;
            }
#else
            // 没有Kconfig配置时，从NVS读取或使用默认值
            topic_ = settings.GetString("topic", "xiaozhi/data");
#endif

            // 如果需要，将Kconfig配置同步到NVS
            if (need_save_config)
            {
                Settings settings_rw("data_mqtt", true);
                settings_rw.SetString("broker", broker_);
                settings_rw.SetInt("port", port_);
                settings_rw.SetString("topic", topic_);
            }
            publish_interval_ms_ = settings.GetInt("interval", MQTT_DATA_PUBLISH_INTERVAL_MS);
            enabled_ = settings.GetInt("enabled", 0) != 0;

            // 加载报警阈值
            LoadThresholds();

            // 如果语音通信使用WebSocket，则可以安全启动MQTT数据客户端
            // 如果有broker配置且enabled为true，则自动启动
            ESP_LOGI(TAG, "MQTT配置检查 - broker: %s, enabled: %" PRId32, settings.GetString("broker").c_str(), settings.GetInt("enabled", 0));
            if (!settings.GetString("broker").empty() && settings.GetInt("enabled", 0) != 0)
            {
                ESP_LOGI(TAG, "自动启动MQTT客户端");
                Start();
            }
            else
            {
                ESP_LOGI(TAG, "MQTT客户端未启动 - broker为空或未启用");
            }
        }

        void SaveConfig()
        {
            Settings settings("data_mqtt", true);
            settings.SetString("broker", broker_);
            settings.SetInt("port", port_);
            settings.SetString("client_id", client_id_);
            settings.SetString("username", username_);
            settings.SetString("password", password_);
            settings.SetString("topic", topic_);
            settings.SetInt("interval", publish_interval_ms_);
            settings.SetInt("enabled", enabled_ ? 1 : 0);
        }

        void Start()
        {
            if (mqtt_task_handle_ != nullptr)
            {
                return;
            }

            if (broker_.empty())
            {
                return;
            }

            enabled_ = true;

            // 创建MQTT数据任务
            xTaskCreate([](void *arg)
                        {
            MqttClient* client = static_cast<MqttClient*>(arg);
            client->MqttTask(); }, "mqtt_data_task", MQTT_DATA_TASK_STACK_SIZE, this, MQTT_DATA_TASK_PRIORITY, &mqtt_task_handle_);
        }

        void Stop()
        {
            if (mqtt_task_handle_ == nullptr)
            {
                return;
            }

            // 通知任务停止
            xEventGroupSetBits(event_group_, MQTT_STOP_BIT);

            // 等待任务结束
            for (int i = 0; i < 10; i++)
            {
                if (mqtt_task_handle_ == nullptr)
                {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            }

            // 如果任务仍然存在，强制删除
            if (mqtt_task_handle_ != nullptr)
            {
                vTaskDelete(mqtt_task_handle_);
                mqtt_task_handle_ = nullptr;
            }

            enabled_ = false;
        }

        void MqttTask()
        {
            // 等待5秒，让其他网络服务先启动完成
            vTaskDelay(pdMS_TO_TICKS(5000));

            // 创建MQTT客户端
            try
            {
                mqtt_client_ = Board::GetInstance().CreateMqtt();
                if (!mqtt_client_)
                {
                    mqtt_task_handle_ = nullptr;
                    vTaskDelete(NULL);
                    return;
                }
            }
            catch (const std::exception &e)
            {
                mqtt_task_handle_ = nullptr;
                vTaskDelete(NULL);
                return;
            }

            // 连接到MQTT服务器
            ConnectToMqttServer();
            std::string control_topic = "control";
            std::string threshold_topic = "threshold";
            mqtt_client_->Subscribe(control_topic);
            mqtt_client_->Subscribe(threshold_topic);

                while (true)
            {
                // 检查是否需要停止
                EventBits_t bits = xEventGroupWaitBits(event_group_, MQTT_STOP_BIT, pdFALSE, pdFALSE, 0);
                if (bits & MQTT_STOP_BIT)
                {
                    break;
                }

                // 检查实际连接状态
                bool is_connected = mqtt_client_ && mqtt_client_->IsConnected();

                // 如果未连接，尝试重新连接
                if (!is_connected)
                {
                    ESP_LOGW(TAG, "MQTT未连接，尝试重新连接...");
                    ConnectToMqttServer();

                    // 等待连接结果，最多等待10秒
                    EventBits_t connect_bits = xEventGroupWaitBits(event_group_,
                                                                   MQTT_CONNECTED_BIT | MQTT_DISCONNECTED_BIT,
                                                                   pdTRUE, pdFALSE,
                                                                   pdMS_TO_TICKS(10000));
                    if (connect_bits & MQTT_CONNECTED_BIT)
                    {
                        ESP_LOGI(TAG, "MQTT重连成功");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "MQTT重连失败，5秒后重试");
                        vTaskDelay(pdMS_TO_TICKS(5000)); // 5秒后重试
                        continue;
                    }
                }

                // 发布数据
                PublishData();

                // 等待下一次发布
                vTaskDelay(pdMS_TO_TICKS(publish_interval_ms_));
            }

            // 断开连接
            if (mqtt_client_)
            {
                mqtt_client_->Disconnect();
                delete mqtt_client_;
                mqtt_client_ = nullptr;
            }

            mqtt_task_handle_ = nullptr;
            vTaskDelete(NULL);
        }

        bool ConnectToMqttServer()
        {
            if (!mqtt_client_)
            {
                ESP_LOGE(TAG, "MQTT客户端为空，无法连接");
                return false;
            }

            ESP_LOGI(TAG, "开始连接MQTT服务器: %s:%d", broker_.c_str(), port_);
            ESP_LOGI(TAG, "连接参数 - client_id: %s, username: %s, password: %s",
                     client_id_.c_str(),
                     username_.empty() ? "(空)" : username_.c_str(),
                     password_.empty() ? "(空)" : "***");

            // 设置回调
            mqtt_client_->OnConnected([this]()
                                      {
            ESP_LOGI(TAG, "MQTT连接成功回调被触发");
            xEventGroupSetBits(event_group_, MQTT_CONNECTED_BIT);

            // 连接成功后订阅控制主题
            std::string control_topic =  "control";
            std::string threshold_topic = "threshold";

            if (mqtt_client_->Subscribe(control_topic)) {
                ESP_LOGI(TAG, "订阅控制主题成功: %s", control_topic.c_str());
            } else {
                ESP_LOGE(TAG, "订阅控制主题失败: %s", control_topic.c_str());
            }

            if (mqtt_client_->Subscribe(threshold_topic)) {
                ESP_LOGI(TAG, "订阅阈值主题成功: %s", threshold_topic.c_str());
            } else {
                ESP_LOGE(TAG, "订阅阈值主题失败: %s", threshold_topic.c_str());
            } });

            mqtt_client_->OnDisconnected([this]()
                                         {
            ESP_LOGW(TAG, "MQTT连接断开回调被触发");
            xEventGroupSetBits(event_group_, MQTT_DISCONNECTED_BIT); });

            // 设置消息接收回调
            mqtt_client_->OnMessage([this](const std::string &topic, const std::string &payload)
                                    { HandleMqttMessage(topic, payload); });

            // 清除之前的连接状态
            xEventGroupClearBits(event_group_, MQTT_CONNECTED_BIT | MQTT_DISCONNECTED_BIT);

            // 连接到服务器
            bool result = mqtt_client_->Connect(broker_, port_, client_id_, username_, password_);
            if (result)
            {
                ESP_LOGI(TAG, "MQTT连接请求发送成功，等待服务器响应...");
                return result; // 返回连接请求是否发送成功，实际连接状态由回调处理
            }
            else
            {
                ESP_LOGE(TAG, "MQTT连接请求发送失败");
                return false;
            }
        }

        // 处理接收到的MQTT消息
        void HandleMqttMessage(const std::string &topic, const std::string &payload)
        {
            ESP_LOGI(TAG, "收到MQTT消息 - 主题: %s, 内容: %s", topic.c_str(), payload.c_str());
            auto &thing_manager = iot::ThingManager::GetInstance();

            // 处理控制消息
            std::string control_topic = "control";
            if (topic == control_topic)
            {
                HandleControlMessage(payload, thing_manager);
            }

            // 处理阈值设置消息
            std::string threshold_topic = "threshold";
            if (topic == threshold_topic)
            {
                HandleThresholdMessage(payload);
            }
        }

        // 处理控制消息
        void HandleControlMessage(const std::string &payload, iot::ThingManager &thing_manager)
        {
            ESP_LOGI(TAG, "处理控制消息: %s", payload.c_str());

            cJSON *root = cJSON_Parse(payload.c_str());
            if (!root)
            {
                ESP_LOGE(TAG, "控制消息JSON解析失败");
                return;
            }

            // 控制风扇
            cJSON *fan = cJSON_GetObjectItem(root, "fan");
            if (cJSON_IsString(fan))
            {
                std::string fan_cmd = fan->valuestring;
                ESP_LOGI(TAG, "控制风扇: %s", fan_cmd.c_str());

                auto fan_thing = thing_manager.GetThingByName("Fan");
                if (fan_thing)
                {
                    if (fan_cmd == "on")
                    {
                        fan_thing->InvokeMethod("turn_on");
                    }
                    else if (fan_cmd == "off")
                    {
                        fan_thing->InvokeMethod("turn_off");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "风扇控制命令无效: %s (应为 on 或 off)", fan_cmd.c_str());
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "未找到风扇设备");
                }
            }

            // 控制窗户
            cJSON *window = cJSON_GetObjectItem(root, "window");
            if (cJSON_IsString(window))
            {
                std::string window_cmd = window->valuestring;
                ESP_LOGI(TAG, "控制窗户: %s", window_cmd.c_str());

                auto window_thing = thing_manager.GetThingByName("WindowController");
                if (window_thing)
                {
                    if (window_cmd == "on")
                    {
                        window_thing->InvokeMethod("Open");
                    }
                    else if (window_cmd == "off")
                    {
                        window_thing->InvokeMethod("Close");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "窗户控制命令无效: %s (应为 on 或 off)", window_cmd.c_str());
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "未找到窗户控制设备");
                }
            }

            // 控制蜂鸣器报警
            cJSON *alarm = cJSON_GetObjectItem(root, "alarm");
            if (cJSON_IsString(alarm))
            {
                std::string alarm_cmd = alarm->valuestring;
                ESP_LOGI(TAG, "控制蜂鸣器报警: %s", alarm_cmd.c_str());

                auto buzzer_thing = thing_manager.GetThingByName("Buzzer");
                if (buzzer_thing)
                {
                    if (alarm_cmd == "on")
                    {
                        buzzer_thing->InvokeMethod("start_alarm");
                    }
                    else if (alarm_cmd == "off")
                    {
                        buzzer_thing->InvokeMethod("stop_alarm");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "蜂鸣器控制命令无效: %s (应为 on 或 off)", alarm_cmd.c_str());
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "未找到蜂鸣器设备");
                }
            }

            // 控制三色灯
            cJSON *light = cJSON_GetObjectItem(root, "light");
            if (cJSON_IsString(light))
            {
                std::string light_cmd = light->valuestring;
                ESP_LOGI(TAG, "控制三色灯: %s", light_cmd.c_str());

                auto light_thing = thing_manager.GetThingByName("3clight");
                if (light_thing)
                {
                    if (light_cmd == "red")
                    {
                        // 先关闭所有颜色，再点亮红色
                        light_thing->InvokeMethod("TurnOffAll");
                        light_thing->InvokeMethod("TurnOnRed");
                    }
                    else if (light_cmd == "green")
                    {
                        // 先关闭所有颜色，再点亮绿色
                        light_thing->InvokeMethod("TurnOffAll");
                        light_thing->InvokeMethod("TurnOnGreen");
                    }
                    else if (light_cmd == "blue")
                    {
                        // 先关闭所有颜色，再点亮蓝色
                        light_thing->InvokeMethod("TurnOffAll");
                        light_thing->InvokeMethod("TurnOnBlue");
                    }
                    else if (light_cmd == "white")
                    {
                        // 点亮所有颜色（白光）
                        light_thing->InvokeMethod("TurnOnAll");
                    }
                    else if (light_cmd == "off")
                    {
                        light_thing->InvokeMethod("TurnOffAll");
                    }
                    else
                    {
                        ESP_LOGW(TAG, "三色灯控制命令无效: %s (应为 red/green/blue/white/off)", light_cmd.c_str());
                    }
                }
                else
                {
                    ESP_LOGW(TAG, "未找到三色灯设备");
                }
            }

            cJSON_Delete(root);
        }

        // 处理阈值设置消息
        void HandleThresholdMessage(const std::string &payload)
        {
            cJSON *root = cJSON_Parse(payload.c_str());
            if (!root)
            {
                return;
            }

            bool need_save = false;

            // 设置甲醛阈值
            cJSON *hcho = cJSON_GetObjectItem(root, "HCHO");
            if (cJSON_IsNumber(hcho))
            {
                hcho_threshold_ = hcho->valueint;
                need_save = true;
            }

            // 设置温度阈值
            cJSON *temp = cJSON_GetObjectItem(root, "TEMP");
            if (cJSON_IsNumber(temp))
            {
                temp_threshold_ = temp->valueint;
                need_save = true;
            }

            // 设置湿度阈值
            cJSON *humi = cJSON_GetObjectItem(root, "HUMI");
            if (cJSON_IsNumber(humi))
            {
                humi_threshold_ = humi->valueint;
                need_save = true;
            }

            // 保存阈值到NVS
            if (need_save)
            {
                SaveThresholds();
            }

            cJSON_Delete(root);
        }

        // 保存阈值到NVS
        void SaveThresholds()
        {
            Settings settings("data_mqtt", true);
            settings.SetInt("hcho_threshold", hcho_threshold_);
            settings.SetInt("temp_threshold", temp_threshold_);
            settings.SetInt("humi_threshold", humi_threshold_);
        }

        // 从NVS加载阈值
        void LoadThresholds()
        {
            Settings settings("data_mqtt", false);
            hcho_threshold_ = settings.GetInt("hcho_threshold", 50);
            temp_threshold_ = settings.GetInt("temp_threshold", 3000);
            humi_threshold_ = settings.GetInt("humi_threshold", 8000);
        }

        void PublishData()
        {
            if (!mqtt_client_ || !mqtt_client_->IsConnected())
            {
                return;
            }

            // 创建JSON数据
            cJSON *root = cJSON_CreateObject();

            // 添加设备信息
            cJSON_AddStringToObject(root, "device_id", client_id_.c_str());
            cJSON_AddStringToObject(root, "mac", SystemInfo::GetMacAddress().c_str());
            cJSON_AddStringToObject(root, "chip", SystemInfo::GetChipModelName().c_str());

            // 添加系统信息
            cJSON *system = cJSON_CreateObject();
            cJSON_AddNumberToObject(system, "free_heap", SystemInfo::GetFreeHeapSize());
            cJSON_AddNumberToObject(system, "min_free_heap", SystemInfo::GetMinimumFreeHeapSize());
            cJSON_AddNumberToObject(system, "flash_size", SystemInfo::GetFlashSize());
            cJSON_AddItemToObject(root, "system", system);

            // 添加电池信息
            int battery_level = 0;
            bool charging = false;
            bool discharging = false;
            if (Board::GetInstance().GetBatteryLevel(battery_level, charging, discharging))
            {
                cJSON *battery = cJSON_CreateObject();
                cJSON_AddNumberToObject(battery, "level", battery_level);
                cJSON_AddBoolToObject(battery, "charging", charging);
                cJSON_AddBoolToObject(battery, "discharging", discharging);
                cJSON_AddItemToObject(root, "battery", battery);
            }

            // 添加温度信息
            float temperature = 0;
            if (Board::GetInstance().GetTemperature(temperature))
            {
                cJSON_AddNumberToObject(root, "chip_temperature", temperature);
            }

            // 从实际传感器获取数据
            auto &thing_manager = iot::ThingManager::GetInstance();

            // 初始化默认值
            int hcho_value = 0;
            int temp_value = 0;
            int humi_value = 0;
            int lx_value = 0;
            int rain_value = 0;
            bool fan_status = false;
            bool window_status = false;
            std::string light_status = "off";

            // 获取甲醛传感器数据
            auto hcho_thing = thing_manager.GetThingByName("HCHOSensor");
            if (hcho_thing)
            {
                auto concentration_property = hcho_thing->GetProperty("hcho_concentration");
                if (concentration_property && concentration_property->type() == kValueTypeNumber)
                {
                    hcho_value = concentration_property->number();
                }
            }

            // 获取DHT11温湿度数据
            auto dht11_thing = thing_manager.GetThingByName("DHT11Sensor");
            if (dht11_thing)
            {
                // 获取温度数据 (转换为百分之一度的整数，如24.02度 -> 2402)
                auto temp_property = dht11_thing->GetProperty("temperature");
                if (temp_property && temp_property->type() == kValueTypeNumber)
                {
                    int dht11_temp = temp_property->number();
                    temp_value = dht11_temp * 100;
                }

                // 获取湿度数据 (转换为百分之一的整数，如59.00% -> 5900)
                auto humidity_property = dht11_thing->GetProperty("humidity");
                if (humidity_property && humidity_property->type() == kValueTypeNumber)
                {
                    int dht11_humidity = humidity_property->number();
                    humi_value = dht11_humidity * 100;
                }
            }

            // 获取光照强度数据
            auto light_thing = thing_manager.GetThingByName("LightSensor");
            if (light_thing)
            {
                auto intensity_property = light_thing->GetProperty("light_intensity");
                if (intensity_property && intensity_property->type() == kValueTypeNumber)
                {
                    lx_value = intensity_property->number();
                }
            }

            // 获取雨量传感器数据
            auto rain_thing = thing_manager.GetThingByName("RainSensor");
            if (rain_thing)
            {
                auto intensity_property = rain_thing->GetProperty("rain_intensity");
                if (intensity_property && intensity_property->type() == kValueTypeNumber)
                {
                    rain_value = intensity_property->number();
                }
            }

            // 获取风扇状态数据
            auto fan_thing = thing_manager.GetThingByName("Fan");
            if (fan_thing)
            {
                auto power_property = fan_thing->GetProperty("power");
                if (power_property && power_property->type() == kValueTypeBoolean)
                {
                    fan_status = power_property->boolean();
                }
            }

            // 获取窗户状态数据
            auto window_thing = thing_manager.GetThingByName("WindowController");
            if (window_thing)
            {
                auto status_property = window_thing->GetProperty("opened");
                if (status_property && status_property->type() == kValueTypeBoolean)
                {
                    window_status = status_property->boolean();
                }
            }

            // 获取三色灯状态数据
            auto colorlight_thing = thing_manager.GetThingByName("3clight");
            if (colorlight_thing)
            {
                auto red_property = colorlight_thing->GetProperty("red");
                auto green_property = colorlight_thing->GetProperty("green");
                auto blue_property = colorlight_thing->GetProperty("blue");

                bool red_on = false, green_on = false, blue_on = false;

                if (red_property && red_property->type() == kValueTypeBoolean)
                {
                    red_on = red_property->boolean();
                }
                if (green_property && green_property->type() == kValueTypeBoolean)
                {
                    green_on = green_property->boolean();
                }
                if (blue_property && blue_property->type() == kValueTypeBoolean)
                {
                    blue_on = blue_property->boolean();
                }

                // 确定当前点亮的颜色（支持5种状态）
                if (red_on && green_on && blue_on)
                {
                    light_status = "white";
                }
                else if (red_on && !green_on && !blue_on)
                {
                    light_status = "red";
                }
                else if (!red_on && green_on && !blue_on)
                {
                    light_status = "green";
                }
                else if (!red_on && !green_on && blue_on)
                {
                    light_status = "blue";
                }
                else
                {
                    // 其他组合或全部关闭，都显示为关闭状态
                    light_status = "off";
                }
            }

            // 构建指定格式的JSON数据: {"HCHO":20,"TEMP":2402,"HUMI":5900,"LX":95,"RAIN":15,"FAN":true,"WINDOW":false,"LIGHT":"red"}
            cJSON_Delete(root);          // 删除之前的复杂JSON结构
            root = cJSON_CreateObject(); // 重新创建简单的JSON结构

            // 添加传感器数据
            cJSON_AddNumberToObject(root, "HCHO", hcho_value);
            cJSON_AddNumberToObject(root, "TEMP", temp_value);
            cJSON_AddNumberToObject(root, "HUMI", humi_value);
            cJSON_AddNumberToObject(root, "LX", lx_value);
            cJSON_AddNumberToObject(root, "RAIN", rain_value);
            cJSON_AddBoolToObject(root, "FAN", fan_status);
            cJSON_AddBoolToObject(root, "WINDOW", window_status);
            cJSON_AddStringToObject(root, "LIGHT", light_status.c_str());
            // 转换为字符串
            char *json_str = cJSON_PrintUnformatted(root);
            std::string payload(json_str);
            cJSON_free(json_str);
            cJSON_Delete(root);

            // 发布数据
            ESP_LOGI(TAG, "发布数据到主题 %s: %s", topic_.c_str(), payload.c_str());
            mqtt_client_->Publish(topic_, payload);
        }
    };

} // namespace iot

DECLARE_THING(MqttClient);