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

#define TAG "MqttClient"
#define MQTT_DATA_TASK_STACK_SIZE 4096
#define MQTT_DATA_TASK_PRIORITY 5
#define MQTT_DATA_PUBLISH_INTERVAL_MS 10000  // 10秒发送一次数据

namespace iot {

class MqttClient : public Thing {
private:
    Mqtt* mqtt_client_ = nullptr;
    TaskHandle_t mqtt_task_handle_ = nullptr;
    EventGroupHandle_t event_group_ = nullptr;
    bool enabled_ = false;
    std::string broker_ = "";
    int port_ = 1883;
    std::string client_id_ = "";
    std::string username_ = "";
    std::string password_ = "";
    std::string topic_ = "xiaozhi/data";
    int publish_interval_ms_ = MQTT_DATA_PUBLISH_INTERVAL_MS;

    // 定义事件位
    static const int MQTT_CONNECTED_BIT = BIT0;
    static const int MQTT_DISCONNECTED_BIT = BIT1;
    static const int MQTT_STOP_BIT = BIT2;

public:
    MqttClient() : Thing("MqttClient", "MQTT客户端，用于发送设备数据") {
        event_group_ = xEventGroupCreate();

        // 定义属性
        properties_.AddBooleanProperty("enabled", "是否启用MQTT客户端", [this]() -> bool {
            return enabled_;
        });
        properties_.AddStringProperty("broker", "MQTT服务器地址", [this]() -> std::string {
            return broker_;
        });
        properties_.AddNumberProperty("port", "MQTT服务器端口", [this]() -> int {
            return port_;
        });
        properties_.AddStringProperty("topic", "MQTT发布主题", [this]() -> std::string {
            return topic_;
        });
        properties_.AddNumberProperty("interval", "数据发布间隔(毫秒)", [this]() -> int {
            return publish_interval_ms_;
        });

        // 定义方法
        ParameterList start_params;
        start_params.AddParameter(Parameter("broker", "MQTT服务器地址", kValueTypeString));
        start_params.AddParameter(Parameter("port", "MQTT服务器端口", kValueTypeNumber, false));
        start_params.AddParameter(Parameter("username", "用户名", kValueTypeString, false));
        start_params.AddParameter(Parameter("password", "密码", kValueTypeString, false));
        start_params.AddParameter(Parameter("topic", "发布主题", kValueTypeString, false));
        start_params.AddParameter(Parameter("interval", "发布间隔(毫秒)", kValueTypeNumber, false));

        methods_.AddMethod("start", "启动MQTT客户端", start_params, [this](const ParameterList& params) {
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
            
            Start();
        });

        ParameterList stop_params;
        methods_.AddMethod("stop", "停止MQTT客户端", stop_params, [this](const ParameterList& params) {
            Stop();
        });

        // 从设置中加载配置
        LoadConfig();
    }

    ~MqttClient() {
        Stop();
        if (event_group_) {
            vEventGroupDelete(event_group_);
        }
    }

private:
    void LoadConfig() {
        Settings settings("data_mqtt", false);
        broker_ = settings.GetString("broker");
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_BROKER)
        if (broker_.empty()) {
            broker_ = CONFIG_MQTT_DATA_CLIENT_BROKER;
        }
#endif
        port_ = settings.GetInt("port", 1883);
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_PORT)
        if (port_ == 0) {
            port_ = CONFIG_MQTT_DATA_CLIENT_PORT;
        }
#endif
        client_id_ = settings.GetString("client_id");
        if (client_id_.empty()) {
            client_id_ = "xiaozhi_" + SystemInfo::GetMacAddress();
            client_id_.erase(std::remove(client_id_.begin(), client_id_.end(), ':'), client_id_.end());
        }
        username_ = settings.GetString("username");
        password_ = settings.GetString("password");
        topic_ = settings.GetString("topic", "xiaozhi/data");
#if CONFIG_MQTT_DATA_CLIENT && defined(CONFIG_MQTT_DATA_CLIENT_TOPIC)
        if (topic_.empty()) {
            topic_ = CONFIG_MQTT_DATA_CLIENT_TOPIC;
        }
#endif
        publish_interval_ms_ = settings.GetInt("interval", MQTT_DATA_PUBLISH_INTERVAL_MS);
        enabled_ = settings.GetInt("enabled", 0) != 0;
        
        // 如果有broker配置且enabled为true，则自动启动
        if (!settings.GetString("broker").empty() && settings.GetInt("enabled", 1) != 0) {
            Start();
        }
    }

    void SaveConfig() {
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

    void Start() {
        if (mqtt_task_handle_ != nullptr) {
            ESP_LOGI(TAG, "MQTT客户端已经在运行");
            return;
        }

        if (broker_.empty()) {
            ESP_LOGE(TAG, "MQTT服务器地址未设置");
            return;
        }

        enabled_ = true;
        SaveConfig();

        // 创建MQTT数据任务
        xTaskCreate([](void* arg) {
            MqttClient* client = static_cast<MqttClient*>(arg);
            client->MqttTask();
        }, "mqtt_data_task", MQTT_DATA_TASK_STACK_SIZE, this, MQTT_DATA_TASK_PRIORITY, &mqtt_task_handle_);

        ESP_LOGI(TAG, "MQTT客户端已启动，连接到 %s:%d", broker_.c_str(), port_);
    }

    void Stop() {
        if (mqtt_task_handle_ == nullptr) {
            return;
        }

        // 通知任务停止
        xEventGroupSetBits(event_group_, MQTT_STOP_BIT);
        
        // 等待任务结束
        for (int i = 0; i < 10; i++) {
            if (mqtt_task_handle_ == nullptr) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        
        // 如果任务仍然存在，强制删除
        if (mqtt_task_handle_ != nullptr) {
            vTaskDelete(mqtt_task_handle_);
            mqtt_task_handle_ = nullptr;
        }

        enabled_ = false;
        SaveConfig();
        ESP_LOGI(TAG, "MQTT客户端已停止");
    }

    void MqttTask() {
        ESP_LOGI(TAG, "MQTT数据任务开始");
        
        // 创建MQTT客户端
        mqtt_client_ = Board::GetInstance().CreateMqtt();
        
        // 连接到MQTT服务器
        bool connected = ConnectToMqttServer();
        
        while (true) {
            // 检查是否需要停止
            EventBits_t bits = xEventGroupWaitBits(event_group_, MQTT_STOP_BIT, pdFALSE, pdFALSE, 0);
            if (bits & MQTT_STOP_BIT) {
                break;
            }
            
            // 如果未连接，尝试重新连接
            if (!connected) {
                connected = ConnectToMqttServer();
                if (!connected) {
                    vTaskDelay(pdMS_TO_TICKS(5000));  // 5秒后重试
                    continue;
                }
            }
            
            // 发布数据
            PublishData();
            
            // 等待下一次发布
            vTaskDelay(pdMS_TO_TICKS(publish_interval_ms_));
        }
        
        // 断开连接
        if (mqtt_client_) {
            mqtt_client_->Disconnect();
            delete mqtt_client_;
            mqtt_client_ = nullptr;
        }
        
        mqtt_task_handle_ = nullptr;
        vTaskDelete(NULL);
    }

    bool ConnectToMqttServer() {
        if (!mqtt_client_) {
            return false;
        }
        
        ESP_LOGI(TAG, "连接到MQTT服务器 %s:%d", broker_.c_str(), port_);
        
        // 设置回调
        mqtt_client_->OnConnected([this]() {
            ESP_LOGI(TAG, "MQTT连接成功");
            xEventGroupSetBits(event_group_, MQTT_CONNECTED_BIT);
        });
        
        mqtt_client_->OnDisconnected([this]() {
            ESP_LOGI(TAG, "MQTT连接断开");
            xEventGroupSetBits(event_group_, MQTT_DISCONNECTED_BIT);
        });
        
        // 连接到服务器
        bool result = mqtt_client_->Connect(broker_, port_, client_id_, username_, password_);
        if (!result) {
            ESP_LOGE(TAG, "MQTT连接失败");
            return false;
        }
        
        return true;
    }

    void PublishData() {
        if (!mqtt_client_ || !mqtt_client_->IsConnected()) {
            ESP_LOGW(TAG, "MQTT未连接，无法发布数据");
            return;
        }
        
        // 创建JSON数据
        cJSON* root = cJSON_CreateObject();
        
        // 添加设备信息
        cJSON_AddStringToObject(root, "device_id", client_id_.c_str());
        cJSON_AddStringToObject(root, "mac", SystemInfo::GetMacAddress().c_str());
        cJSON_AddStringToObject(root, "chip", SystemInfo::GetChipModelName().c_str());
        
        // 添加系统信息
        cJSON* system = cJSON_CreateObject();
        cJSON_AddNumberToObject(system, "free_heap", SystemInfo::GetFreeHeapSize());
        cJSON_AddNumberToObject(system, "min_free_heap", SystemInfo::GetMinimumFreeHeapSize());
        cJSON_AddNumberToObject(system, "flash_size", SystemInfo::GetFlashSize());
        cJSON_AddItemToObject(root, "system", system);
        
        // 添加电池信息
        int battery_level = 0;
        bool charging = false;
        bool discharging = false;
        if (Board::GetInstance().GetBatteryLevel(battery_level, charging, discharging)) {
            cJSON* battery = cJSON_CreateObject();
            cJSON_AddNumberToObject(battery, "level", battery_level);
            cJSON_AddBoolToObject(battery, "charging", charging);
            cJSON_AddBoolToObject(battery, "discharging", discharging);
            cJSON_AddItemToObject(root, "battery", battery);
        }
        
        // 添加温度信息
        float temperature = 0;
        if (Board::GetInstance().GetTemperature(temperature)) {
            cJSON_AddNumberToObject(root, "chip_temperature", temperature);
        }

        // 添加DHT11温湿度数据
        #if 0
        auto& thing_manager = iot::ThingManager::GetInstance();
        auto dht11_thing = thing_manager.GetThing("DHT11Sensor");
        if (dht11_thing) {
            cJSON* dht11_data = cJSON_CreateObject();

            // 获取温度数据
            auto temp_property = dht11_thing->GetProperty("temperature");
            if (temp_property && temp_property->type() == kValueTypeNumber) {
                float dht11_temp = temp_property->number();
                cJSON_AddNumberToObject(dht11_data, "temperature", dht11_temp);
            }

            // 获取湿度数据
            auto humidity_property = dht11_thing->GetProperty("humidity");
            if (humidity_property && humidity_property->type() == kValueTypeNumber) {
                float dht11_humidity = humidity_property->number();
                cJSON_AddNumberToObject(dht11_data, "humidity", dht11_humidity);
            }

            // 获取数据有效性
            auto valid_property = dht11_thing->GetProperty("data_valid");
            if (valid_property && valid_property->type() == kValueTypeBoolean) {
                bool data_valid = valid_property->boolean();
                cJSON_AddBoolToObject(dht11_data, "data_valid", data_valid);
            }

            cJSON_AddItemToObject(root, "dht11", dht11_data);
        }

        // 添加光照强度数据
        auto light_thing = thing_manager.GetThing("LightSensor");
        if (light_thing) {
            cJSON* light_data = cJSON_CreateObject();

            // 获取光照强度数据
            auto intensity_property = light_thing->GetProperty("light_intensity");
            if (intensity_property && intensity_property->type() == kValueTypeNumber) {
                int light_intensity = intensity_property->number();
                cJSON_AddNumberToObject(light_data, "intensity", light_intensity);
            }

            // 获取ADC原始值
            auto raw_property = light_thing->GetProperty("raw_adc_value");
            if (raw_property && raw_property->type() == kValueTypeNumber) {
                int raw_adc = raw_property->number();
                cJSON_AddNumberToObject(light_data, "raw_adc", raw_adc);
            }

            // 获取数据有效性
            auto valid_property = light_thing->GetProperty("data_valid");
            if (valid_property && valid_property->type() == kValueTypeBoolean) {
                bool data_valid = valid_property->boolean();
                cJSON_AddBoolToObject(light_data, "data_valid", data_valid);
            }

            cJSON_AddItemToObject(root, "light", light_data);
        }

        // 添加雨量传感器数据
        auto rain_thing = thing_manager.GetThing("RainSensor");
        if (rain_thing) {
            cJSON* rain_data = cJSON_CreateObject();

            // 获取雨量强度数据
            auto intensity_property = rain_thing->GetProperty("rain_intensity");
            if (intensity_property && intensity_property->type() == kValueTypeNumber) {
                int rain_intensity = intensity_property->number();
                cJSON_AddNumberToObject(rain_data, "intensity", rain_intensity);
            }

            // 获取ADC原始值
            auto raw_property = rain_thing->GetProperty("raw_adc_value");
            if (raw_property && raw_property->type() == kValueTypeNumber) {
                int raw_adc = raw_property->number();
                cJSON_AddNumberToObject(rain_data, "raw_adc", raw_adc);
            }

            // 获取数据有效性
            auto valid_property = rain_thing->GetProperty("data_valid");
            if (valid_property && valid_property->type() == kValueTypeBoolean) {
                bool data_valid = valid_property->boolean();
                cJSON_AddBoolToObject(rain_data, "data_valid", data_valid);
            }

            cJSON_AddItemToObject(root, "rain", rain_data);
        }

        // 添加甲醛传感器数据
        auto hcho_thing = thing_manager.GetThing("HCHOSensor");
        if (hcho_thing) {
            cJSON* hcho_data = cJSON_CreateObject();

            // 获取甲醛浓度数据
            auto concentration_property = hcho_thing->GetProperty("hcho_concentration");
            if (concentration_property && concentration_property->type() == kValueTypeNumber) {
                float hcho_concentration = concentration_property->number();
                cJSON_AddNumberToObject(hcho_data, "concentration", hcho_concentration);
            }

            // 获取原始值
            auto raw_property = hcho_thing->GetProperty("raw_value");
            if (raw_property && raw_property->type() == kValueTypeNumber) {
                int raw_value = raw_property->number();
                cJSON_AddNumberToObject(hcho_data, "raw_value", raw_value);
            }

            // 获取数据有效性
            auto valid_property = hcho_thing->GetProperty("data_valid");
            if (valid_property && valid_property->type() == kValueTypeBoolean) {
                bool data_valid = valid_property->boolean();
                cJSON_AddBoolToObject(hcho_data, "data_valid", data_valid);
            }

            cJSON_AddItemToObject(root, "hcho", hcho_data);
        }
        #endif
        // 转换为字符串
        char* json_str = cJSON_PrintUnformatted(root);
        std::string payload(json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);
        
        // 发布数据
        ESP_LOGI(TAG, "发布数据到主题 %s: %s", topic_.c_str(), payload.c_str());
        if (!mqtt_client_->Publish(topic_, payload)) {
            ESP_LOGE(TAG, "发布数据失败");
        }
    }
};

} // namespace iot

DECLARE_THING(MqttClient); 