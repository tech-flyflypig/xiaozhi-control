#include "iot/thing.h"
#include "board.h"
#include "application.h"
#include "esp_log.h"
#include <mqtt.h>
#include <cJSON.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG = "WindowController";

// MQTT主题
#define MQTT_WINDOW_COMMAND_TOPIC "xiaozhi/window/command"
#define MQTT_WINDOW_STATE_TOPIC "xiaozhi/window/state"

// 定义步进电机控制引脚
#define STEPPER_DIR_PIN  GPIO_NUM_19   // 方向控制引脚
#define STEPPER_STEP_PIN GPIO_NUM_18   // 步进控制引脚
#define STEPPER_EN_PIN   GPIO_NUM_21   // 使能控制引脚

// 定义步进电机参数
#define STEPPER_FULL_REVOLUTION_STEPS 200  // 步进电机一圈步数(通常是200步/圈)
#define WINDOW_FULL_STEPS 1000              // 窗户开关所需步数
#define DEFAULT_STEP_DELAY_MS 2            // 默认步进延迟(ms)

namespace iot {

class WindowController : public Thing {
private:
    // MQTT相关
    bool mqtt_available_ = false;     // 是否找到了MqttClient
    std::string mqtt_topic_ = MQTT_WINDOW_COMMAND_TOPIC;
    
    // 步进电机控制相关
    gpio_num_t dir_pin_;     // 方向引脚
    gpio_num_t step_pin_;    // 步进引脚
    gpio_num_t en_pin_;      // 使能引脚
    
    bool is_enabled_ = false;    // 电机是否使能
    bool is_running_ = false;    // 电机是否正在运行
    bool direction_ = true;      // 电机方向(true为正向，用于打开窗户)
    int position_ = 0;           // 当前位置(步数)
    int target_position_ = 0;    // 目标位置(步数)
    int step_delay_ms_ = DEFAULT_STEP_DELAY_MS;  // 步进延迟时间(ms)
    bool window_opened_ = false; // 窗户状态(false:关闭, true:打开)
    
    TaskHandle_t stepper_task_handle_ = nullptr;  // 步进电机任务句柄
    
    // 初始化GPIO
    void InitializeGpio() {
        // 配置方向引脚
        gpio_config_t dir_config;
        dir_config.pin_bit_mask = (1ULL << dir_pin_);
        dir_config.mode = GPIO_MODE_OUTPUT;
        dir_config.pull_up_en = GPIO_PULLUP_DISABLE;
        dir_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        dir_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&dir_config);
        
        // 配置步进引脚
        gpio_config_t step_config;
        step_config.pin_bit_mask = (1ULL << step_pin_);
        step_config.mode = GPIO_MODE_OUTPUT;
        step_config.pull_up_en = GPIO_PULLUP_DISABLE;
        step_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        step_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&step_config);
        
        // 配置使能引脚
        gpio_config_t en_config;
        en_config.pin_bit_mask = (1ULL << en_pin_);
        en_config.mode = GPIO_MODE_OUTPUT;
        en_config.pull_up_en = GPIO_PULLUP_DISABLE;
        en_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
        en_config.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&en_config);
        
        // 初始状态：电机未使能，步进和方向引脚为低电平
        gpio_set_level(en_pin_, 1);    // 大多数驱动器低电平有效，所以设为1禁用电机
        gpio_set_level(dir_pin_, 0);
        gpio_set_level(step_pin_, 0);
        
        ESP_LOGI(TAG, "步进电机GPIO初始化完成 - DIR:%d, STEP:%d, EN:%d", 
                 dir_pin_, step_pin_, en_pin_);
    }

    // 更新电机使能状态
    void UpdateEnableState() {
        gpio_set_level(en_pin_, is_enabled_ ? 0 : 1);  // 假设低电平有效
    }

    // 更新方向状态
    void UpdateDirectionState() {
        gpio_set_level(dir_pin_, direction_ ? 1 : 0);
    }

    // 执行单步
    void Step() {
        gpio_set_level(step_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(1));  // 短暂延时
        gpio_set_level(step_pin_, 0);
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms_));  // 步进延时
        
        // 更新位置
        if (direction_) {
            position_++;
        } else {
            position_--;
        }
    }

    // 步进电机控制任务
    static void StepperTaskFunc(void* arg) {
        WindowController* controller = static_cast<WindowController*>(arg);
        controller->RunStepperTask();
        vTaskDelete(nullptr);
    }

    void RunStepperTask() {
        while (is_running_) {
            if (position_ == target_position_) {
                // 到达目标位置，停止运行
                is_running_ = false;
                ESP_LOGI(TAG, "窗户到达目标位置: %d", position_);
                break;
            }
            
            // 设置方向
            direction_ = (target_position_ > position_);
            UpdateDirectionState();
            
            // 执行一步
            Step();
            
            // 更新窗户状态
            UpdateWindowState();
        }
        
        // 任务结束，禁用电机
        is_enabled_ = false;
        UpdateEnableState();
        stepper_task_handle_ = nullptr;
        
        // 发布最新状态
        PublishWindowState();
    }

    // 更新窗户状态
    void UpdateWindowState() {
        window_opened_ = (position_ >= WINDOW_FULL_STEPS / 2);
    }

    // 启动电机运行
    void StartMotor(bool open) {
        if (is_running_) {
            ESP_LOGW(TAG, "窗户正在移动，无法启动新操作");
            return;
        }
        
        // 设置目标位置
        target_position_ = open ? WINDOW_FULL_STEPS : 0;
        
        // 如果已经在目标状态，无需操作
        if ((open && window_opened_ && position_ == WINDOW_FULL_STEPS) ||
            (!open && !window_opened_ && position_ == 0)) {
            ESP_LOGI(TAG, "窗户已经在%s状态", open ? "打开" : "关闭");
            return;
        }
        
        // 使能电机
        is_enabled_ = true;
        UpdateEnableState();
        
        // 设置运行标志
        is_running_ = true;
        
        // 创建步进电机控制任务
        xTaskCreate(StepperTaskFunc, "window_task", 4096, this, 5, &stepper_task_handle_);
        
        ESP_LOGI(TAG, "开始%s窗户", open ? "打开" : "关闭");
    }
    
    // 发布窗户状态到MQTT
    void PublishWindowState() {
        if (!mqtt_available_) {
            return;
        }
        
        // 创建状态JSON
        cJSON* root = cJSON_CreateObject();
        cJSON_AddStringToObject(root, "device", "xiaozhi_window");
        
        // 添加窗户状态
        cJSON_AddStringToObject(root, "state", window_opened_ ? "opened" : "closed");
        
        // 添加电机位置
        cJSON_AddNumberToObject(root, "position", position_);
        
        // 转换为字符串
        char* json_str = cJSON_PrintUnformatted(root);
        std::string payload(json_str);
        cJSON_free(json_str);
        cJSON_Delete(root);
        
        // 发布状态
        ESP_LOGI(TAG, "发布窗户状态: %s", payload.c_str());
        
        // 通过MqttClient发布状态
        Thing* mqtt_thing = Application::GetInstance().GetThingManager().GetThingByName("MqttClient");
        if (mqtt_thing) {
            // 使用MqttClient的Publish方法发布状态
            ParameterList params;
            params.AddParameter(Parameter("topic", MQTT_WINDOW_STATE_TOPIC, kValueTypeString));
            params.AddParameter(Parameter("message", payload, kValueTypeString));
            mqtt_thing->GetMethods()["Publish"].Invoke(params);
        }
    }
    
    // 处理MQTT命令消息
    void HandleMqttCommand(const std::string& message) {
        ESP_LOGI(TAG, "收到MQTT窗户控制命令: %s", message.c_str());
        
        // 解析JSON消息
        cJSON* root = cJSON_Parse(message.c_str());
        if (!root) {
            ESP_LOGE(TAG, "解析JSON失败");
            return;
        }
        
        // 获取命令类型
        cJSON* cmd = cJSON_GetObjectItem(root, "command");
        if (!cmd || !cJSON_IsString(cmd)) {
            ESP_LOGE(TAG, "无效的命令格式");
            cJSON_Delete(root);
            return;
        }
        
        std::string command = cmd->valuestring;
        
        // 执行相应命令
        if (command == "open") {
            // 打开窗户
            StartMotor(true);
        } else if (command == "close") {
            // 关闭窗户
            StartMotor(false);
        } else if (command == "stop") {
            // 停止电机
            is_running_ = false;
            ESP_LOGI(TAG, "停止窗户移动");
        } else if (command == "toggle") {
            // 切换窗户状态
            StartMotor(!window_opened_);
            ESP_LOGI(TAG, "切换窗户状态: %s", window_opened_ ? "关闭" : "打开");
        } else {
            ESP_LOGE(TAG, "未知命令: %s", command.c_str());
        }
        
        cJSON_Delete(root);
    }
    
    // 初始化MQTT
    void InitializeMqtt() {
        // 查找现有的MQTT客户端Thing
        Thing* mqtt_thing = Application::GetInstance().GetThingManager().GetThingByName("MqttClient");
        if (mqtt_thing) {
            mqtt_available_ = true;
            ESP_LOGI(TAG, "找到MqttClient Thing，将使用它处理MQTT通信");
            
            // 订阅窗户控制命令主题
            ParameterList subscribe_params;
            subscribe_params.AddParameter(Parameter("topic", mqtt_topic_, kValueTypeString));
            subscribe_params.AddParameter(Parameter("callback", "HandleWindowCommand", kValueTypeString));
            
            try {
                mqtt_thing->GetMethods()["Subscribe"].Invoke(subscribe_params);
                ESP_LOGI(TAG, "已通过MqttClient订阅窗户控制主题: %s", mqtt_topic_.c_str());
                
                // 发布初始状态
                PublishWindowState();
            } catch (const std::exception& e) {
                ESP_LOGE(TAG, "通过MqttClient订阅主题失败: %s", e.what());
                mqtt_available_ = false;
            }
        } else {
            ESP_LOGW(TAG, "未找到MqttClient Thing，MQTT功能不可用");
            mqtt_available_ = false;
        }
    }

public:
    WindowController(gpio_num_t dir_pin = STEPPER_DIR_PIN, 
                    gpio_num_t step_pin = STEPPER_STEP_PIN, 
                    gpio_num_t en_pin = STEPPER_EN_PIN) 
        : Thing("window_controller", "窗户控制器，支持语音控制和MQTT控制"),
          dir_pin_(dir_pin),
          step_pin_(step_pin),
          en_pin_(en_pin) {
        
        // 初始化GPIO
        InitializeGpio();
        
        // 初始化MQTT
        InitializeMqtt();
        
        // 定义窗户状态属性
        properties_.AddBooleanProperty("running", "窗户是否正在移动", [this]() -> bool {
            return is_running_;
        });
        
        properties_.AddNumberProperty("position", "当前位置(步数)", [this]() -> int {
            return position_;
        });
        
        properties_.AddBooleanProperty("opened", "窗户是否打开", [this]() -> bool {
            return window_opened_;
        });
        
        properties_.AddBooleanProperty("mqtt_available", "MQTT功能是否可用", [this]() -> bool {
            return mqtt_available_;
        });
        
        properties_.AddStringProperty("mqtt_topic", "MQTT命令主题", [this]() -> std::string {
            return mqtt_topic_;
        });
        
        // 定义MQTT相关方法
        methods_.AddMethod("PublishState", "发布窗户状态到MQTT", ParameterList(), 
                          [this](const ParameterList& params) {
            PublishWindowState();
        });
        
        // MQTT命令处理回调，由MqttClient调用
        methods_.AddMethod("HandleWindowCommand", "处理来自MQTT的窗户控制命令", ParameterList({
            Parameter("topic", "MQTT主题", kValueTypeString, true),
            Parameter("message", "MQTT消息内容", kValueTypeString, true)
        }), [this](const ParameterList& params) {
            std::string message = params["message"].string();
            HandleMqttCommand(message);
        });
        
        methods_.AddMethod("SetMqttTopic", "设置MQTT命令主题", ParameterList({
            Parameter("topic", "MQTT主题", kValueTypeString, true)
        }), [this](const ParameterList& params) {
            std::string new_topic = params["topic"].string();
            
            // 如果MQTT可用，更新订阅
            if (mqtt_available_) {
                Thing* mqtt_thing = Application::GetInstance().GetThingManager().GetThingByName("MqttClient");
                if (mqtt_thing) {
                    // 取消订阅旧主题
                    ParameterList unsubscribe_params;
                    unsubscribe_params.AddParameter(Parameter("topic", mqtt_topic_, kValueTypeString));
                    mqtt_thing->GetMethods()["Unsubscribe"].Invoke(unsubscribe_params);
                    
                    // 订阅新主题
                    ParameterList subscribe_params;
                    subscribe_params.AddParameter(Parameter("topic", new_topic, kValueTypeString));
                    subscribe_params.AddParameter(Parameter("callback", "HandleWindowCommand", kValueTypeString));
                    mqtt_thing->GetMethods()["Subscribe"].Invoke(subscribe_params);
                    
                    ESP_LOGI(TAG, "MQTT主题已更新为: %s", new_topic.c_str());
                }
            }
            
            mqtt_topic_ = new_topic;
        });
        
        // 定义窗户控制方法（作为语音控制的接口）
        methods_.AddMethod("OpenWindow", "打开窗户", ParameterList(), 
                          [this](const ParameterList& params) {
            StartMotor(true);
            ESP_LOGI(TAG, "打开窗户");
        });
        
        methods_.AddMethod("CloseWindow", "关闭窗户", ParameterList(), 
                          [this](const ParameterList& params) {
            StartMotor(false);
            ESP_LOGI(TAG, "关闭窗户");
        });
        
        methods_.AddMethod("ToggleWindow", "切换窗户状态", ParameterList(), 
                          [this](const ParameterList& params) {
            StartMotor(!window_opened_);
            ESP_LOGI(TAG, "切换窗户状态: %s", window_opened_ ? "关闭" : "打开");
        });
        
        methods_.AddMethod("StopWindow", "停止窗户移动", ParameterList(), 
                          [this](const ParameterList& params) {
            // 停止电机
            is_running_ = false;
            ESP_LOGI(TAG, "停止窗户移动");
        });
        
        methods_.AddMethod("SetSpeed", "设置窗户移动速度", ParameterList({
            Parameter("speed", "移动速度(1-10，越大越慢)", kValueTypeNumber, true)
        }), [this](const ParameterList& params) {
            int speed = params["speed"].number();
            if (speed < 1) speed = 1;
            if (speed > 10) speed = 10;
            
            // 将速度转换为延迟时间 (1->1ms, 10->10ms)
            step_delay_ms_ = speed;
            ESP_LOGI(TAG, "窗户移动速度已设置，步进延迟: %d ms", step_delay_ms_);
        });
    }
    
    ~WindowController() {
        // 停止电机任务
        if (stepper_task_handle_ != nullptr) {
            is_running_ = false;
            vTaskDelay(pdMS_TO_TICKS(100));  // 等待任务结束
            if (stepper_task_handle_ != nullptr) {
                vTaskDelete(stepper_task_handle_);
                stepper_task_handle_ = nullptr;
            }
        }
        
        // 取消MQTT订阅
        if (mqtt_available_) {
            Thing* mqtt_thing = Application::GetInstance().GetThingManager().GetThingByName("MqttClient");
            if (mqtt_thing) {
                ParameterList unsubscribe_params;
                unsubscribe_params.AddParameter(Parameter("topic", mqtt_topic_, kValueTypeString));
                mqtt_thing->GetMethods()["Unsubscribe"].Invoke(unsubscribe_params);
            }
        }
    }
};

} // namespace iot

// 注册设备到系统
DECLARE_THING(WindowController); 