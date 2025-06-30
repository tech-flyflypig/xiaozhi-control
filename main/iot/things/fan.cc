#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "Fan"

// 风扇控制GPIO引脚配置
#ifdef CONFIG_IDF_TARGET_ESP32
#define FAN_CONTROL_GPIO_PIN GPIO_NUM_12
#else
#define FAN_CONTROL_GPIO_PIN GPIO_NUM_38
#endif

namespace iot {

class Fan : public Thing {
private:
    gpio_num_t gpio_pin_;
    bool fan_power_;        // 风扇状态 (true: 开启, false: 关闭)
    bool auto_mode_;        // 自动模式开关
    int temp_threshold_;    // 温度阈值 (°C * 100)
    SemaphoreHandle_t control_mutex_;
    
    // 初始化GPIO
    void InitializeGpio() {
        gpio_config_t config = {
            .pin_bit_mask = (1ULL << gpio_pin_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));
        
        // 初始状态：关闭风扇 (低电平)
        gpio_set_level(gpio_pin_, 0);
        fan_power_ = false;
        
        ESP_LOGI(TAG, "风扇GPIO初始化完成，控制引脚: %d, 初始状态: 关闭", gpio_pin_);
    }
    
    // 设置风扇状态
    void SetFanState(bool power_on) {
        if (xSemaphoreTake(control_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            fan_power_ = power_on;
            gpio_set_level(gpio_pin_, power_on ? 1 : 0);
            ESP_LOGI(TAG, "风扇状态已设置为: %s", power_on ? "开启" : "关闭");
            xSemaphoreGive(control_mutex_);
        }
    }

    // 自动控制风扇（基于温度）
    void AutoControlFan(float temperature) {
        if (!auto_mode_) return;

        int temp_int = (int)(temperature * 100);
        if (temp_int > temp_threshold_ && !fan_power_) {
            SetFanState(true);
            ESP_LOGI(TAG, "自动模式: 温度%.2f°C超过阈值%.2f°C，自动打开风扇",
                     temperature, temp_threshold_ / 100.0f);
        } else if (temp_int <= (temp_threshold_ - 200) && fan_power_) {  // 2°C滞后
            SetFanState(false);
            ESP_LOGI(TAG, "自动模式: 温度%.2f°C低于阈值%.2f°C，自动关闭风扇",
                     temperature, (temp_threshold_ - 200) / 100.0f);
        }
    }
    
    // 获取风扇状态
    bool GetFanState() {
        bool state = false;
        if (xSemaphoreTake(control_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            state = fan_power_;
            xSemaphoreGive(control_mutex_);
        }
        return state;
    }

public:
    Fan() : Thing("Fan", "智能风扇，支持开关控制和温度自动控制"),
            gpio_pin_(FAN_CONTROL_GPIO_PIN), fan_power_(false), auto_mode_(false), temp_threshold_(2800) {
        
        // 创建互斥锁
        control_mutex_ = xSemaphoreCreateMutex();
        if (control_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建控制互斥锁失败");
            return;
        }
        
        // 初始化GPIO
        InitializeGpio();
        
        // 定义设备属性
        properties_.AddBooleanProperty("power", "风扇是否开启", [this]() -> bool {
            return GetFanState();
        });
        
        properties_.AddStringProperty("status", "风扇状态描述", [this]() -> std::string {
            return GetFanState() ? "运行中" : "已停止";
        });

        properties_.AddBooleanProperty("auto_mode", "自动模式开关", [this]() -> bool {
            return auto_mode_;
        });

        properties_.AddNumberProperty("temp_threshold", "温度阈值(°C*100)", [this]() -> int {
            return temp_threshold_;
        });
        
        // 定义设备方法
        methods_.AddMethod("turn_on", "打开风扇", ParameterList(), [this](const ParameterList& parameters) {
            SetFanState(true);
        });
        
        methods_.AddMethod("turn_off", "关闭风扇", ParameterList(), [this](const ParameterList& parameters) {
            SetFanState(false);
        });
        
        methods_.AddMethod("toggle", "切换风扇状态", ParameterList(), [this](const ParameterList& parameters) {
            bool current_state = GetFanState();
            SetFanState(!current_state);
        });
        
        // 添加带参数的设置方法
        ParameterList power_params;
        power_params.AddParameter(Parameter("power", "风扇开关状态", kValueTypeBoolean, true));
        methods_.AddMethod("set_power", "设置风扇开关状态", power_params, [this](const ParameterList& parameters) {
            try {
                bool power_value = parameters["power"].boolean();
                SetFanState(power_value);
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置风扇状态失败：%s", e.what());
            }
        });

        // 自动模式控制方法
        methods_.AddMethod("enable_auto_mode", "启用自动模式", ParameterList(), [this](const ParameterList& parameters) {
            auto_mode_ = true;
            ESP_LOGI(TAG, "风扇自动模式已启用");
        });

        methods_.AddMethod("disable_auto_mode", "禁用自动模式", ParameterList(), [this](const ParameterList& parameters) {
            auto_mode_ = false;
            ESP_LOGI(TAG, "风扇自动模式已禁用");
        });

        // 设置温度阈值方法
        methods_.AddMethod("set_temp_threshold", "设置温度阈值",
                          ParameterList({Parameter("threshold", "温度阈值(°C*100)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int threshold = parameters["threshold"].number();
                if (threshold > 0 && threshold <= 5000) {  // 0-50°C范围
                    temp_threshold_ = threshold;
                    ESP_LOGI(TAG, "风扇温度阈值已设置为: %.2f°C", threshold / 100.0f);
                } else {
                    ESP_LOGW(TAG, "温度阈值超出范围: 0-50°C");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置温度阈值失败：%s", e.what());
            }
        });

        // 手动温度控制方法（供自动化控制器调用）
        methods_.AddMethod("auto_control", "基于温度自动控制",
                          ParameterList({Parameter("temperature", "当前温度(°C*100)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int temp_int = parameters["temperature"].number();
                float temperature = temp_int / 100.0f; // 转换为浮点数温度
                AutoControlFan(temperature);
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "自动控制失败：%s", e.what());
            }
        });
        
        ESP_LOGI(TAG, "风扇设备初始化完成，GPIO引脚: %d", gpio_pin_);
    }
    
    ~Fan() {
        // 关闭风扇
        if (control_mutex_) {
            SetFanState(false);
            vSemaphoreDelete(control_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(Fan);
