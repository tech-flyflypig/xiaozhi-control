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
    bool fan_power_;  // 风扇状态 (true: 开启, false: 关闭)
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
    Fan() : Thing("Fan", "智能风扇，支持开关控制"), 
            gpio_pin_(FAN_CONTROL_GPIO_PIN), fan_power_(false) {
        
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
