#include "iot/thing.h"
#include "iot/thing_manager.h"
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

// 定义ULN2003驱动的步进电机控制引脚
#define STEPPER_IN1_PIN  GPIO_NUM_17   // ULN2003 IN1
#define STEPPER_IN2_PIN  GPIO_NUM_18   // ULN2003 IN2
#define STEPPER_IN3_PIN  GPIO_NUM_19   // ULN2003 IN3
#define STEPPER_IN4_PIN  GPIO_NUM_20  // ULN2003 IN4

// 定义步进电机参数
#define STEPPER_FULL_REVOLUTION_STEPS 4096  // 28BYJ-48步进电机一圈步数(通常是4096步/圈)
#define WINDOW_FULL_STEPS 4096              // 窗户开关所需步数
#define DEFAULT_STEP_DELAY_MS 2            // 默认步进延迟(ms)

namespace iot {

class WindowController : public Thing {
private:
    // MQTT相关
    bool mqtt_available_ = false;     // 是否找到了MqttClient
    std::string mqtt_topic_ = MQTT_WINDOW_COMMAND_TOPIC;
    
    // 步进电机控制相关
    gpio_num_t in1_pin_;     // IN1引脚
    gpio_num_t in2_pin_;     // IN2引脚
    gpio_num_t in3_pin_;     // IN3引脚
    gpio_num_t in4_pin_;     // IN4引脚
    
    // 步进序列 - 半步模式(8步序列)
    const uint8_t step_sequence[8][4] = {
        {1, 0, 0, 0},
        {1, 1, 0, 0},
        {0, 1, 0, 0},
        {0, 1, 1, 0},
        {0, 0, 1, 0},
        {0, 0, 1, 1},
        {0, 0, 0, 1},
        {1, 0, 0, 1}
    };
    
    int current_step_ = 0;    // 当前步进序列索引
    bool is_running_ = false; // 电机是否正在运行
    bool direction_ = true;   // 电机方向(true为正向，用于打开窗户)
    int position_ = 0;        // 当前位置(步数)
    int target_position_ = 0; // 目标位置(步数)
    int step_delay_ms_ = DEFAULT_STEP_DELAY_MS;  // 步进延迟时间(ms)
    bool window_opened_ = false; // 窗户状态(false:关闭, true:打开)
    
    TaskHandle_t stepper_task_handle_ = nullptr;  // 步进电机任务句柄
    
    // 初始化GPIO
    void InitializeGpio() {
        // 配置四个控制引脚
        gpio_config_t io_conf = {};
        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        
        // 配置IN1引脚
        io_conf.pin_bit_mask = (1ULL << in1_pin_);
        gpio_config(&io_conf);
        
        // 配置IN2引脚
        io_conf.pin_bit_mask = (1ULL << in2_pin_);
        gpio_config(&io_conf);
        
        // 配置IN3引脚
        io_conf.pin_bit_mask = (1ULL << in3_pin_);
        gpio_config(&io_conf);
        
        // 配置IN4引脚
        io_conf.pin_bit_mask = (1ULL << in4_pin_);
        gpio_config(&io_conf);
        
        // 初始状态：所有引脚低电平
        gpio_set_level(in1_pin_, 0);
        gpio_set_level(in2_pin_, 0);
        gpio_set_level(in3_pin_, 0);
        gpio_set_level(in4_pin_, 0);
        
        ESP_LOGI(TAG, "ULN2003步进电机GPIO初始化完成 - IN1:%d, IN2:%d, IN3:%d, IN4:%d", 
                 in1_pin_, in2_pin_, in3_pin_, in4_pin_);
        
        // 运行简单的电机测试
        TestMotor();
    }
    
    // 测试电机功能
    void TestMotor() {
        ESP_LOGI(TAG, "开始测试电机功能...");
        
        // 先测试单个线圈激活
        ESP_LOGI(TAG, "测试线圈1...");
        gpio_set_level(in1_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(in1_pin_, 0);
        
        ESP_LOGI(TAG, "测试线圈2...");
        gpio_set_level(in2_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(in2_pin_, 0);
        
        ESP_LOGI(TAG, "测试线圈3...");
        gpio_set_level(in3_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(in3_pin_, 0);
        
        ESP_LOGI(TAG, "测试线圈4...");
        gpio_set_level(in4_pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(in4_pin_, 0);
        
        // 测试简单的步进序列
        ESP_LOGI(TAG, "测试步进序列...");
        for (int i = 0; i < 32; i++) {
            current_step_ = i % 8;
            gpio_set_level(in1_pin_, step_sequence[current_step_][0]);
            gpio_set_level(in2_pin_, step_sequence[current_step_][1]);
            gpio_set_level(in3_pin_, step_sequence[current_step_][2]);
            gpio_set_level(in4_pin_, step_sequence[current_step_][3]);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        
        // 停止电机
        StopMotor();
        ESP_LOGI(TAG, "电机测试完成");
    }
    
    // 停止电机
    void StopMotor() {
        // 断电所有线圈
        gpio_set_level(in1_pin_, 0);
        gpio_set_level(in2_pin_, 0);
        gpio_set_level(in3_pin_, 0);
        gpio_set_level(in4_pin_, 0);
    }

    // 执行单步
    void Step() {
        // 根据方向更新步进序列索引
        if (direction_) {
            current_step_ = (current_step_ + 1) % 8;
        } else {
            current_step_ = (current_step_ + 7) % 8;  // +7相当于-1，但避免负数
        }
        
        // 输出步进序列
        gpio_set_level(in1_pin_, step_sequence[current_step_][0]);
        gpio_set_level(in2_pin_, step_sequence[current_step_][1]);
        gpio_set_level(in3_pin_, step_sequence[current_step_][2]);
        gpio_set_level(in4_pin_, step_sequence[current_step_][3]);
        
        // 每100步打印一次日志
        if (position_ % 100 == 0) {
            ESP_LOGI(TAG, "步进电机移动中 - 位置: %d, 方向: %s, 步进序列: [%d,%d,%d,%d]", 
                    position_, direction_ ? "正向" : "反向", 
                    step_sequence[current_step_][0],
                    step_sequence[current_step_][1],
                    step_sequence[current_step_][2],
                    step_sequence[current_step_][3]);
        }
        
        // 延时
        vTaskDelay(pdMS_TO_TICKS(step_delay_ms_));
        
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
        ESP_LOGI(TAG, "步进电机任务开始执行 - 初始位置: %d, 目标位置: %d", position_, target_position_);
        
        while (is_running_) {
            if (position_ == target_position_) {
                // 到达目标位置，停止运行
                is_running_ = false;
                ESP_LOGI(TAG, "窗户到达目标位置: %d", position_);
                break;
            }
            
            // 设置方向
            direction_ = (target_position_ > position_);
            
            // 执行一步
            Step();
            
            // 更新窗户状态
            UpdateWindowState();
        }
        
        // 任务结束，断电电机
        StopMotor();
        ESP_LOGI(TAG, "步进电机任务完成 - 最终位置: %d, 窗户状态: %s", 
                position_, window_opened_ ? "打开" : "关闭");
        
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
        
        ESP_LOGI(TAG, "启动电机请求 - 当前位置: %d, 目标位置: %d, 当前状态: %s", 
                 position_, target_position_, window_opened_ ? "打开" : "关闭");
        
        // 如果已经在目标状态，无需操作
        if ((open && window_opened_ && position_ == WINDOW_FULL_STEPS) ||
            (!open && !window_opened_ && position_ == 0)) {
            ESP_LOGI(TAG, "窗户已经在%s状态", open ? "打开" : "关闭");
            return;
        }
        
        // 设置运行标志
        is_running_ = true;
        
        // 创建步进电机控制任务
        BaseType_t result = xTaskCreate(StepperTaskFunc, "window_task", 4096, this, 5, &stepper_task_handle_);
        
        if (result == pdPASS) {
            ESP_LOGI(TAG, "开始%s窗户, 步进电机任务创建成功", open ? "打开" : "关闭");
        } else {
            ESP_LOGE(TAG, "步进电机任务创建失败, 错误码: %d", result);
            is_running_ = false;
        }
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
        Thing* mqtt_thing = iot::ThingManager::GetInstance().GetThingByName("MqttClient");
        if (mqtt_thing) {
            // 使用MqttClient的Publish方法发布状态
            // 创建命令JSON对象
            cJSON* command = cJSON_CreateObject();
            cJSON_AddStringToObject(command, "method", "Publish");
            cJSON* params = cJSON_CreateObject();
            cJSON_AddStringToObject(params, "topic", MQTT_WINDOW_STATE_TOPIC);
            cJSON_AddStringToObject(params, "message", payload.c_str());
            cJSON_AddItemToObject(command, "parameters", params);
            mqtt_thing->Invoke(command);
            cJSON_Delete(command);
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
        Thing* mqtt_thing = iot::ThingManager::GetInstance().GetThingByName("MqttClient");
        if (mqtt_thing) {
            mqtt_available_ = true;
            ESP_LOGI(TAG, "找到MqttClient Thing，将使用它处理MQTT通信");
            
            // 订阅窗户控制命令主题
            try {
                // 创建订阅命令
                cJSON* command = cJSON_CreateObject();
                cJSON_AddStringToObject(command, "method", "Subscribe");
                cJSON* params = cJSON_CreateObject();
                cJSON_AddStringToObject(params, "topic", mqtt_topic_.c_str());
                cJSON_AddStringToObject(params, "callback", "HandleWindowCommand");
                cJSON_AddItemToObject(command, "parameters", params);
                
                // 执行命令
                mqtt_thing->Invoke(command);
                cJSON_Delete(command);
                
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
    WindowController(gpio_num_t in1_pin = STEPPER_IN1_PIN, 
                    gpio_num_t in2_pin = STEPPER_IN2_PIN, 
                    gpio_num_t in3_pin = STEPPER_IN3_PIN, 
                    gpio_num_t in4_pin = STEPPER_IN4_PIN) 
        : Thing("window_controller", "窗户控制器，支持语音控制和MQTT控制"),
          in1_pin_(in1_pin),
          in2_pin_(in2_pin),
          in3_pin_(in3_pin),
          in4_pin_(in4_pin) {
        
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
                Thing* mqtt_thing = iot::ThingManager::GetInstance().GetThingByName("MqttClient");
                if (mqtt_thing) {
                    // 取消订阅旧主题
                    cJSON* unsubscribe_command = cJSON_CreateObject();
                    cJSON_AddStringToObject(unsubscribe_command, "method", "Unsubscribe");
                    cJSON* unsubscribe_params = cJSON_CreateObject();
                    cJSON_AddStringToObject(unsubscribe_params, "topic", mqtt_topic_.c_str());
                    cJSON_AddItemToObject(unsubscribe_command, "parameters", unsubscribe_params);
                    mqtt_thing->Invoke(unsubscribe_command);
                    cJSON_Delete(unsubscribe_command);
                    
                    // 订阅新主题
                    cJSON* subscribe_command = cJSON_CreateObject();
                    cJSON_AddStringToObject(subscribe_command, "method", "Subscribe");
                    cJSON* subscribe_params = cJSON_CreateObject();
                    cJSON_AddStringToObject(subscribe_params, "topic", new_topic.c_str());
                    cJSON_AddStringToObject(subscribe_params, "callback", "HandleWindowCommand");
                    cJSON_AddItemToObject(subscribe_command, "parameters", subscribe_params);
                    mqtt_thing->Invoke(subscribe_command);
                    cJSON_Delete(subscribe_command);
                    
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
        
        // 断电电机
        StopMotor();
        
        // 取消MQTT订阅
        if (mqtt_available_) {
            Thing* mqtt_thing = iot::ThingManager::GetInstance().GetThingByName("MqttClient");
            if (mqtt_thing) {
                cJSON* unsubscribe_command = cJSON_CreateObject();
                cJSON_AddStringToObject(unsubscribe_command, "method", "Unsubscribe");
                cJSON* unsubscribe_params = cJSON_CreateObject();
                cJSON_AddStringToObject(unsubscribe_params, "topic", mqtt_topic_.c_str());
                cJSON_AddItemToObject(unsubscribe_command, "parameters", unsubscribe_params);
                mqtt_thing->Invoke(unsubscribe_command);
                cJSON_Delete(unsubscribe_command);
            }
        }
    }
};

} // namespace iot

// 注册设备到系统
DECLARE_THING(WindowController); 