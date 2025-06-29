#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "Buzzer"

// 蜂鸣器控制GPIO引脚配置
#define BUZZER_CONTROL_GPIO_PIN GPIO_NUM_14

namespace iot {

class Buzzer : public Thing {
private:
    gpio_num_t gpio_pin_;
    bool buzzer_power_;  // 蜂鸣器状态 (true: 开启, false: 关闭)
    SemaphoreHandle_t control_mutex_;
    esp_timer_handle_t auto_off_timer_;  // 自动关闭定时器
    esp_timer_handle_t alarm_timer_;     // 报警模式定时器
    bool is_alarm_mode_;                 // 是否处于报警模式
    int alarm_beep_count_;               // 报警蜂鸣次数计数
    
    // 初始化GPIO（有源蜂鸣器）
    void InitializeGpio() {
        // 配置GPIO为输出模式
        gpio_config_t config = {
            .pin_bit_mask = (1ULL << gpio_pin_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&config));

        // 初始状态：关闭蜂鸣器（低电平）
        gpio_set_level(gpio_pin_, 0);
        buzzer_power_ = false;
        is_alarm_mode_ = false;
        alarm_beep_count_ = 0;

        ESP_LOGI(TAG, "有源蜂鸣器GPIO初始化完成，控制引脚: %d, 初始状态: 关闭(低电平)", gpio_pin_);
    }
    
    // 设置蜂鸣器状态（有源蜂鸣器，高电平触发）
    void SetBuzzerState(bool power_on) {
        if (xSemaphoreTake(control_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            buzzer_power_ = power_on;
            // 有源蜂鸣器：高电平开启，低电平关闭
            gpio_set_level(gpio_pin_, power_on ? 1 : 0);
            ESP_LOGI(TAG, "蜂鸣器状态已设置为: %s", power_on ? "开启(高电平)" : "关闭(低电平)");
            xSemaphoreGive(control_mutex_);
        }
    }
    
    // 获取蜂鸣器状态
    bool GetBuzzerState() {
        bool state = false;
        if (xSemaphoreTake(control_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            state = buzzer_power_;
            xSemaphoreGive(control_mutex_);
        }
        return state;
    }
    
    // 自动关闭定时器回调
    static void AutoOffTimerCallback(void* arg) {
        Buzzer* buzzer = static_cast<Buzzer*>(arg);
        buzzer->SetBuzzerState(false);
        ESP_LOGI(TAG, "蜂鸣器自动关闭");
    }

    // 报警定时器回调
    static void AlarmTimerCallback(void* arg) {
        Buzzer* buzzer = static_cast<Buzzer*>(arg);
        buzzer->HandleAlarmTick();
    }

    // 处理报警节拍
    void HandleAlarmTick() {
        if (!is_alarm_mode_) {
            return;
        }

        if (alarm_beep_count_ < 3) {  // 连续蜂鸣3次
            if (buzzer_power_) {
                // 当前是开启状态，关闭蜂鸣器
                SetBuzzerState(false);
                // 100ms后再次触发
                esp_timer_start_once(alarm_timer_, 100 * 1000);
            } else {
                // 当前是关闭状态，开启蜂鸣器
                SetBuzzerState(true);
                alarm_beep_count_++;
                // 200ms后再次触发
                esp_timer_start_once(alarm_timer_, 200 * 1000);
            }
        } else {
            // 完成3次蜂鸣，停止报警
            SetBuzzerState(false);
            is_alarm_mode_ = false;
            alarm_beep_count_ = 0;
            ESP_LOGI(TAG, "报警结束");
        }
    }
    
    // 蜂鸣指定时间
    void BeepForDuration(int duration_ms) {
        // 停止之前的定时器
        if (auto_off_timer_) {
            esp_timer_stop(auto_off_timer_);
        }

        // 停止报警模式
        StopAlarm();

        // 开启蜂鸣器
        SetBuzzerState(true);

        // 设置自动关闭定时器
        if (duration_ms > 0) {
            esp_timer_start_once(auto_off_timer_, duration_ms * 1000);  // 转换为微秒
        }
    }

    // 启动报警模式
    void StartAlarm() {
        // 停止之前的定时器
        if (auto_off_timer_) {
            esp_timer_stop(auto_off_timer_);
        }
        if (alarm_timer_) {
            esp_timer_stop(alarm_timer_);
        }

        is_alarm_mode_ = true;
        alarm_beep_count_ = 0;

        // 立即开始第一次蜂鸣
        SetBuzzerState(true);
        alarm_beep_count_ = 1;

        // 200ms后触发下一次
        esp_timer_start_once(alarm_timer_, 200 * 1000);

        ESP_LOGI(TAG, "启动报警模式");
    }

    // 停止报警模式
    void StopAlarm() {
        if (is_alarm_mode_) {
            is_alarm_mode_ = false;
            alarm_beep_count_ = 0;
            if (alarm_timer_) {
                esp_timer_stop(alarm_timer_);
            }
            ESP_LOGI(TAG, "停止报警模式");
        }
    }

public:
    Buzzer() : Thing("Buzzer", "智能有源蜂鸣器，GPIO驱动，支持开关控制、定时蜂鸣和报警模式"),
               gpio_pin_(BUZZER_CONTROL_GPIO_PIN), buzzer_power_(false),
               is_alarm_mode_(false), alarm_beep_count_(0) {

        // 创建互斥锁
        control_mutex_ = xSemaphoreCreateMutex();
        if (control_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建控制互斥锁失败");
            return;
        }

        // 初始化GPIO
        InitializeGpio();

        // 创建自动关闭定时器
        esp_timer_create_args_t timer_args = {
            .callback = AutoOffTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "buzzer_auto_off",
            .skip_unhandled_events = true
        };
        esp_err_t ret = esp_timer_create(&timer_args, &auto_off_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建自动关闭定时器失败: %s", esp_err_to_name(ret));
        }

        // 创建报警定时器
        esp_timer_create_args_t alarm_timer_args = {
            .callback = AlarmTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "buzzer_alarm",
            .skip_unhandled_events = true
        };
        ret = esp_timer_create(&alarm_timer_args, &alarm_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建报警定时器失败: %s", esp_err_to_name(ret));
        }
        
        // 定义设备属性
        properties_.AddBooleanProperty("power", "蜂鸣器是否开启", [this]() -> bool {
            return GetBuzzerState();
        });
        
        properties_.AddStringProperty("status", "蜂鸣器状态描述", [this]() -> std::string {
            return GetBuzzerState() ? "蜂鸣中" : "静音";
        });
        
        // 定义设备方法
        methods_.AddMethod("turn_on", "打开蜂鸣器", ParameterList(), [this](const ParameterList& parameters) {
            SetBuzzerState(true);
        });
        
        methods_.AddMethod("turn_off", "关闭蜂鸣器", ParameterList(), [this](const ParameterList& parameters) {
            SetBuzzerState(false);
            // 停止自动关闭定时器
            if (auto_off_timer_) {
                esp_timer_stop(auto_off_timer_);
            }
        });
        
        methods_.AddMethod("toggle", "切换蜂鸣器状态", ParameterList(), [this](const ParameterList& parameters) {
            bool current_state = GetBuzzerState();
            SetBuzzerState(!current_state);
        });
        
        // 添加带参数的设置方法
        ParameterList power_params;
        power_params.AddParameter(Parameter("power", "蜂鸣器开关状态", kValueTypeBoolean, true));
        methods_.AddMethod("set_power", "设置蜂鸣器开关状态", power_params, [this](const ParameterList& parameters) {
            try {
                bool power_value = parameters["power"].boolean();
                SetBuzzerState(power_value);
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置蜂鸣器状态失败：%s", e.what());
            }
        });
        
        // 添加定时蜂鸣方法
        ParameterList beep_params;
        beep_params.AddParameter(Parameter("duration", "蜂鸣持续时间(毫秒)", kValueTypeNumber, true));
        methods_.AddMethod("beep", "蜂鸣指定时间后自动关闭", beep_params, [this](const ParameterList& parameters) {
            try {
                int duration = parameters["duration"].number();
                if (duration > 0 && duration <= 60000) {  // 限制最大60秒
                    BeepForDuration(duration);
                } else {
                    ESP_LOGW(TAG, "蜂鸣时间无效，应在1-60000毫秒之间");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置蜂鸣时间失败：%s", e.what());
            }
        });
        
        // 添加快捷蜂鸣方法
        methods_.AddMethod("beep_short", "短蜂鸣(200毫秒)", ParameterList(), [this](const ParameterList& parameters) {
            BeepForDuration(200);
        });
        
        methods_.AddMethod("beep_long", "长蜂鸣(1000毫秒)", ParameterList(), [this](const ParameterList& parameters) {
            BeepForDuration(1000);
        });

        // 添加报警相关方法
        methods_.AddMethod("start_alarm", "启动报警模式(连续3次短蜂鸣)", ParameterList(), [this](const ParameterList& parameters) {
            StartAlarm();
        });

        methods_.AddMethod("stop_alarm", "停止报警模式", ParameterList(), [this](const ParameterList& parameters) {
            StopAlarm();
        });

        ESP_LOGI(TAG, "蜂鸣器设备初始化完成，GPIO引脚: %d", gpio_pin_);
    }
    
    ~Buzzer() {
        // 停止报警模式
        StopAlarm();

        // 关闭蜂鸣器
        if (control_mutex_) {
            SetBuzzerState(false);
        }

        // 清理定时器
        if (auto_off_timer_) {
            esp_timer_stop(auto_off_timer_);
            esp_timer_delete(auto_off_timer_);
        }

        if (alarm_timer_) {
            esp_timer_stop(alarm_timer_);
            esp_timer_delete(alarm_timer_);
        }

        // 清理互斥锁
        if (control_mutex_) {
            vSemaphoreDelete(control_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(Buzzer);
