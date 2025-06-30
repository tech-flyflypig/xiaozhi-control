#include "iot/thing.h"
#include "iot/thing_manager.h"
#include "board.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "AutomationController"

// 报警冷却间隔 (10秒)
#define ALARM_COOLDOWN_MS 10000

// 检查间隔 (1秒)
#define CHECK_INTERVAL_MS 1000

namespace iot {

class AutomationController : public Thing {
private:
    // 传感器阈值配置 - 支持上下阈值
    int temp_threshold_high_;     // 温度上阈值 (°C * 100)
    int temp_threshold_low_;      // 温度下阈值 (°C * 100)
    int humi_threshold_high_;     // 湿度上阈值 (% * 100)
    int humi_threshold_low_;      // 湿度下阈值 (% * 100)
    int hcho_threshold_high_;     // 甲醛上阈值 (μg/m³)
    int hcho_threshold_low_;      // 甲醛下阈值 (μg/m³)
    int light_threshold_high_;    // 光照上阈值 (%)
    int light_threshold_low_;     // 光照下阈值 (%)
    int rain_threshold_high_;     // 雨量上阈值 (%)
    int rain_threshold_low_;      // 雨量下阈值 (%)

    // 联动配置 - 简化为设备自动模式控制
    int fan_temp_threshold_;      // 风扇温度联动阈值 (°C * 100)
    int window_rain_threshold_;   // 窗户雨量联动阈值 (%)
    
    // 报警状态
    bool temp_alarm_active_;
    bool humi_alarm_active_;
    bool hcho_alarm_active_;
    bool light_alarm_active_;
    bool rain_alarm_active_;
    int64_t last_temp_alarm_time_;
    int64_t last_humi_alarm_time_;
    int64_t last_hcho_alarm_time_;
    int64_t last_light_alarm_time_;
    int64_t last_rain_alarm_time_;
    
    // 定时器和互斥锁
    esp_timer_handle_t check_timer_;
    SemaphoreHandle_t config_mutex_;
    
    // 初始化配置
    void InitializeConfig() {
        LoadConfig();
        ESP_LOGI(TAG, "自动化控制器配置加载完成");
        ESP_LOGI(TAG, "温度阈值: %.2f-%.2f°C, 湿度阈值: %.2f-%.2f%%, 甲醛阈值: %d-%d μg/m³",
                 temp_threshold_low_ / 100.0f, temp_threshold_high_ / 100.0f,
                 humi_threshold_low_ / 100.0f, humi_threshold_high_ / 100.0f,
                 hcho_threshold_low_, hcho_threshold_high_);
        ESP_LOGI(TAG, "光照阈值: %d-%d%%, 雨量阈值: %d-%d%%",
                 light_threshold_low_, light_threshold_high_,
                 rain_threshold_low_, rain_threshold_high_);
        ESP_LOGI(TAG, "风扇温度联动阈值: %.2f°C, 窗户雨量联动阈值: %d%%",
                 fan_temp_threshold_ / 100.0f, window_rain_threshold_);
    }
    
    // 从NVS加载配置
    void LoadConfig() {
        Settings settings("automation", false);

        // 加载传感器阈值 - 上下阈值
        temp_threshold_high_ = settings.GetInt("temp_high", 3000);    // 默认30°C
        temp_threshold_low_ = settings.GetInt("temp_low", 1000);      // 默认10°C
        humi_threshold_high_ = settings.GetInt("humi_high", 8000);    // 默认80%
        humi_threshold_low_ = settings.GetInt("humi_low", 2000);      // 默认20%
        hcho_threshold_high_ = settings.GetInt("hcho_high", 100);     // 默认100 μg/m³
        hcho_threshold_low_ = settings.GetInt("hcho_low", 10);        // 默认10 μg/m³
        light_threshold_high_ = settings.GetInt("light_high", 80);    // 默认80%
        light_threshold_low_ = settings.GetInt("light_low", 20);      // 默认20%
        rain_threshold_high_ = settings.GetInt("rain_high", 70);      // 默认70%
        rain_threshold_low_ = settings.GetInt("rain_low", 10);        // 默认10%

        // 加载联动阈值
        fan_temp_threshold_ = settings.GetInt("fan_temp", 2800);      // 默认28°C
        window_rain_threshold_ = settings.GetInt("window_rain", 30);  // 默认30%
    }
    
    // 保存配置到NVS
    void SaveConfig() {
        if (xSemaphoreTake(config_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            Settings settings("automation", true);

            // 保存传感器阈值
            settings.SetInt("temp_high", temp_threshold_high_);
            settings.SetInt("temp_low", temp_threshold_low_);
            settings.SetInt("humi_high", humi_threshold_high_);
            settings.SetInt("humi_low", humi_threshold_low_);
            settings.SetInt("hcho_high", hcho_threshold_high_);
            settings.SetInt("hcho_low", hcho_threshold_low_);
            settings.SetInt("light_high", light_threshold_high_);
            settings.SetInt("light_low", light_threshold_low_);
            settings.SetInt("rain_high", rain_threshold_high_);
            settings.SetInt("rain_low", rain_threshold_low_);

            // 保存联动阈值
            settings.SetInt("fan_temp", fan_temp_threshold_);
            settings.SetInt("window_rain", window_rain_threshold_);

            xSemaphoreGive(config_mutex_);
            ESP_LOGI(TAG, "自动化控制器配置已保存");
        }
    }
    
    // 定时器回调函数
    static void CheckTimerCallback(void* arg) {
        AutomationController* controller = static_cast<AutomationController*>(arg);
        controller->PerformCheck();
    }
    
    // 执行检查操作
    void PerformCheck() {
        CheckAlarms();
        CheckLinkages();
    }
    
    // 检查报警条件
    void CheckAlarms() {
        auto& thing_manager = ThingManager::GetInstance();
        int64_t current_time = esp_timer_get_time() / 1000;  // 转换为毫秒
        
        // 检查温度报警
        auto dht11_sensor = thing_manager.GetThingByName("DHT11Sensor");
        if (dht11_sensor) {
            auto temp_prop = dht11_sensor->GetProperty("temperature");
            if (temp_prop && temp_prop->type() == kValueTypeNumber) {
                float temp = temp_prop->number();
                int temp_int = (int)(temp * 100);  // 转换为整数比较

                if (temp_int > temp_threshold_high_ || temp_int < temp_threshold_low_) {
                    if (!temp_alarm_active_ || (current_time - last_temp_alarm_time_) > ALARM_COOLDOWN_MS) {
                        TriggerTemperatureAlarm(temp, temp_int > temp_threshold_high_);
                        temp_alarm_active_ = true;
                        last_temp_alarm_time_ = current_time;
                    }
                } else {
                    temp_alarm_active_ = false;
                }
            }
        }
        
        // 检查湿度报警
        if (dht11_sensor) {
            auto humi_prop = dht11_sensor->GetProperty("humidity");
            if (humi_prop && humi_prop->type() == kValueTypeNumber) {
                float humi = humi_prop->number();
                int humi_int = (int)(humi * 100);  // 转换为整数比较

                if (humi_int > humi_threshold_high_ || humi_int < humi_threshold_low_) {
                    if (!humi_alarm_active_ || (current_time - last_humi_alarm_time_) > ALARM_COOLDOWN_MS) {
                        TriggerHumidityAlarm(humi, humi_int > humi_threshold_high_);
                        humi_alarm_active_ = true;
                        last_humi_alarm_time_ = current_time;
                    }
                } else {
                    humi_alarm_active_ = false;
                }
            }
        }
        
        // 检查甲醛报警
        auto hcho_sensor = thing_manager.GetThingByName("HCHOSensor");
        if (hcho_sensor) {
            auto hcho_prop = hcho_sensor->GetProperty("hcho_level");
            if (hcho_prop && hcho_prop->type() == kValueTypeNumber) {
                int hcho_level = hcho_prop->number();

                if (hcho_level > hcho_threshold_high_ || hcho_level < hcho_threshold_low_) {
                    if (!hcho_alarm_active_ || (current_time - last_hcho_alarm_time_) > ALARM_COOLDOWN_MS) {
                        TriggerHCHOAlarm(hcho_level, hcho_level > hcho_threshold_high_);
                        hcho_alarm_active_ = true;
                        last_hcho_alarm_time_ = current_time;
                    }
                } else {
                    hcho_alarm_active_ = false;
                }
            }
        }
    }
    
    // 检查联动条件 - 使用设备内部自动模式
    void CheckLinkages() {
        auto& thing_manager = ThingManager::GetInstance();

        // 温度-风扇联动：通过风扇的自动控制方法
        auto dht11_sensor = thing_manager.GetThingByName("DHT11Sensor");
        auto fan = thing_manager.GetThingByName("Fan");

        if (dht11_sensor && fan) {
            auto temp_prop = dht11_sensor->GetProperty("temperature");
            if (temp_prop && temp_prop->type() == kValueTypeNumber) {
                float temp = temp_prop->number();
                // 创建包含温度参数的参数列表
                Parameter temp_param("temperature", "当前温度(°C)", kValueTypeNumber, true);
                temp_param.set_number((int)(temp * 100)); // 风扇内部会除以100转换回浮点数
                ParameterList params({temp_param});
                fan->InvokeMethod("auto_control", params);
            }
        }

        // 雨量-窗户联动：通过窗户的自动控制方法
        auto rain_sensor = thing_manager.GetThingByName("RainSensor");
        auto window = thing_manager.GetThingByName("WindowController");

        if (rain_sensor && window) {
            auto rain_prop = rain_sensor->GetProperty("rain_intensity");
            if (rain_prop && rain_prop->type() == kValueTypeNumber) {
                int rain_intensity = rain_prop->number();
                // 创建包含雨量参数的参数列表
                Parameter rain_param("rain_intensity", "当前雨量强度(%)", kValueTypeNumber, true);
                rain_param.set_number(rain_intensity);
                ParameterList params({rain_param});
                window->InvokeMethod("auto_control", params);
            }
        }
    }
    
    // 触发温度报警
    void TriggerTemperatureAlarm(float temperature, bool is_high) {
        if (is_high) {
            ESP_LOGW(TAG, "温度报警: 当前温度%.2f°C超过上阈值%.2f°C", temperature, temp_threshold_high_ / 100.0f);
        } else {
            ESP_LOGW(TAG, "温度报警: 当前温度%.2f°C低于下阈值%.2f°C", temperature, temp_threshold_low_ / 100.0f);
        }

        // 触发蜂鸣器报警
        auto& thing_manager = ThingManager::GetInstance();
        auto buzzer = thing_manager.GetThingByName("Buzzer");
        if (buzzer) {
            buzzer->InvokeMethod("start_alarm");
        }
    }

    // 触发湿度报警
    void TriggerHumidityAlarm(float humidity, bool is_high) {
        if (is_high) {
            ESP_LOGW(TAG, "湿度报警: 当前湿度%.2f%%超过上阈值%.2f%%", humidity, humi_threshold_high_ / 100.0f);
        } else {
            ESP_LOGW(TAG, "湿度报警: 当前湿度%.2f%%低于下阈值%.2f%%", humidity, humi_threshold_low_ / 100.0f);
        }

        // 触发蜂鸣器报警
        auto& thing_manager = ThingManager::GetInstance();
        auto buzzer = thing_manager.GetThingByName("Buzzer");
        if (buzzer) {
            buzzer->InvokeMethod("start_alarm");
        }
    }

    // 触发甲醛报警
    void TriggerHCHOAlarm(int hcho_level, bool is_high) {
        if (is_high) {
            ESP_LOGW(TAG, "甲醛报警: 当前甲醛浓度%d μg/m³超过上阈值%d μg/m³", hcho_level, hcho_threshold_high_);
        } else {
            ESP_LOGW(TAG, "甲醛报警: 当前甲醛浓度%d μg/m³低于下阈值%d μg/m³", hcho_level, hcho_threshold_low_);
        }

        // 触发蜂鸣器报警
        auto& thing_manager = ThingManager::GetInstance();
        auto buzzer = thing_manager.GetThingByName("Buzzer");
        if (buzzer) {
            buzzer->InvokeMethod("start_alarm");
        }
    }

public:
    AutomationController() : Thing("AutomationController", "自动化控制器，支持报警和联动功能"),
                            temp_threshold_high_(3000), temp_threshold_low_(1000),
                            humi_threshold_high_(8000), humi_threshold_low_(2000),
                            hcho_threshold_high_(100), hcho_threshold_low_(10),
                            light_threshold_high_(80), light_threshold_low_(20),
                            rain_threshold_high_(70), rain_threshold_low_(10),
                            fan_temp_threshold_(2800), window_rain_threshold_(30),
                            temp_alarm_active_(false), humi_alarm_active_(false), hcho_alarm_active_(false),
                            light_alarm_active_(false), rain_alarm_active_(false),
                            last_temp_alarm_time_(0), last_humi_alarm_time_(0), last_hcho_alarm_time_(0),
                            last_light_alarm_time_(0), last_rain_alarm_time_(0) {
        
        // 创建互斥锁
        config_mutex_ = xSemaphoreCreateMutex();
        if (config_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建配置互斥锁失败");
            return;
        }
        
        // 初始化配置
        InitializeConfig();
        
        // 定义设备属性
        properties_.AddNumberProperty("temp_threshold_high", "温度报警上阈值(°C*100)", [this]() -> int {
            return temp_threshold_high_;
        });

        properties_.AddNumberProperty("temp_threshold_low", "温度报警下阈值(°C*100)", [this]() -> int {
            return temp_threshold_low_;
        });

        properties_.AddNumberProperty("humi_threshold_high", "湿度报警上阈值(%*100)", [this]() -> int {
            return humi_threshold_high_;
        });

        properties_.AddNumberProperty("humi_threshold_low", "湿度报警下阈值(%*100)", [this]() -> int {
            return humi_threshold_low_;
        });

        properties_.AddNumberProperty("hcho_threshold_high", "甲醛报警上阈值(μg/m³)", [this]() -> int {
            return hcho_threshold_high_;
        });

        properties_.AddNumberProperty("hcho_threshold_low", "甲醛报警下阈值(μg/m³)", [this]() -> int {
            return hcho_threshold_low_;
        });

        properties_.AddNumberProperty("fan_temp_threshold", "风扇温度联动阈值(°C*100)", [this]() -> int {
            return fan_temp_threshold_;
        });

        properties_.AddNumberProperty("window_rain_threshold", "窗户雨量联动阈值(%)", [this]() -> int {
            return window_rain_threshold_;
        });
        
        properties_.AddBooleanProperty("temp_alarm_active", "温度报警状态", [this]() -> bool {
            return temp_alarm_active_;
        });
        
        properties_.AddBooleanProperty("humi_alarm_active", "湿度报警状态", [this]() -> bool {
            return humi_alarm_active_;
        });
        
        properties_.AddBooleanProperty("hcho_alarm_active", "甲醛报警状态", [this]() -> bool {
            return hcho_alarm_active_;
        });

        // 定义设备方法
        // 传感器报警阈值设置方法 - 上阈值
        methods_.AddMethod("set_temp_threshold_high", "设置温度报警上阈值",
                          ParameterList({Parameter("threshold", "温度上阈值(°C*100)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int threshold = parameters["threshold"].number();
                if (threshold > 0 && threshold <= 10000) {  // 0-100°C范围
                    temp_threshold_high_ = threshold;
                    SaveConfig();
                    ESP_LOGI(TAG, "温度报警上阈值已设置为: %.2f°C", threshold / 100.0f);
                } else {
                    ESP_LOGW(TAG, "温度上阈值超出范围: 0-100°C");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置温度上阈值失败：%s", e.what());
            }
        });

        methods_.AddMethod("set_temp_threshold_low", "设置温度报警下阈值",
                          ParameterList({Parameter("threshold", "温度下阈值(°C*100)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int threshold = parameters["threshold"].number();
                if (threshold > 0 && threshold <= 10000) {  // 0-100°C范围
                    temp_threshold_low_ = threshold;
                    SaveConfig();
                    ESP_LOGI(TAG, "温度报警下阈值已设置为: %.2f°C", threshold / 100.0f);
                } else {
                    ESP_LOGW(TAG, "温度下阈值超出范围: 0-100°C");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置温度下阈值失败：%s", e.what());
            }
        });

        // 注意：旧的方法已移除，因为我们现在使用上下阈值系统

        // 设置联动阈值方法
        methods_.AddMethod("set_fan_temp_threshold", "设置风扇温度联动阈值",
                          ParameterList({Parameter("threshold", "温度阈值(°C*100)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int threshold = parameters["threshold"].number();
                if (threshold > 0 && threshold <= 5000) {  // 0-50°C范围
                    fan_temp_threshold_ = threshold;
                    SaveConfig();
                    ESP_LOGI(TAG, "风扇温度联动阈值已设置为: %.2f°C", threshold / 100.0f);
                } else {
                    ESP_LOGW(TAG, "温度阈值超出范围: 0-50°C");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置风扇温度阈值失败：%s", e.what());
            }
        });

        methods_.AddMethod("set_window_rain_threshold", "设置窗户雨量联动阈值",
                          ParameterList({Parameter("threshold", "雨量阈值(%)", kValueTypeNumber, true)}),
                          [this](const ParameterList& parameters) {
            try {
                int threshold = parameters["threshold"].number();
                if (threshold > 0 && threshold <= 100) {  // 0-100%范围
                    window_rain_threshold_ = threshold;
                    SaveConfig();
                    ESP_LOGI(TAG, "窗户雨量联动阈值已设置为: %d%%", threshold);
                } else {
                    ESP_LOGW(TAG, "雨量阈值超出范围: 0-100%%");
                }
            } catch (const std::exception& e) {
                ESP_LOGW(TAG, "设置窗户雨量阈值失败：%s", e.what());
            }
        });

        // 手动检查方法
        methods_.AddMethod("check_now", "立即执行检查", ParameterList(),
                          [this](const ParameterList& parameters) {
            PerformCheck();
            ESP_LOGI(TAG, "手动检查已执行");
        });

        // 重置报警状态方法
        methods_.AddMethod("reset_alarms", "重置所有报警状态", ParameterList(),
                          [this](const ParameterList& parameters) {
            temp_alarm_active_ = false;
            humi_alarm_active_ = false;
            hcho_alarm_active_ = false;
            ESP_LOGI(TAG, "所有报警状态已重置");
        });

        // 创建定时器，每秒检查一次
        esp_timer_create_args_t timer_args = {
            .callback = CheckTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "automation_check_timer",
            .skip_unhandled_events = true
        };

        esp_err_t ret = esp_timer_create(&timer_args, &check_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
            return;
        }

        // 启动定时器
        ret = esp_timer_start_periodic(check_timer_, CHECK_INTERVAL_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
            return;
        }

        ESP_LOGI(TAG, "自动化控制器初始化完成，检查间隔: %dms", CHECK_INTERVAL_MS);
    }

    ~AutomationController() {
        if (check_timer_) {
            esp_timer_stop(check_timer_);
            esp_timer_delete(check_timer_);
        }
        if (config_mutex_) {
            vSemaphoreDelete(config_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(AutomationController);
