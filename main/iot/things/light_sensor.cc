#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <driver/adc.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "LightSensor"

// 光敏电阻 GPIO 引脚配置 (ADC)
#define LIGHT_SENSOR_GPIO_PIN GPIO_NUM_13
#define LIGHT_SENSOR_ADC_CHANNEL ADC_CHANNEL_4  // GPIO13 对应 ADC1_CHANNEL_4

// 数据读取间隔 (5秒)
#define LIGHT_READ_INTERVAL_MS 5000

namespace iot {

class LightSensor : public Thing {
private:
    gpio_num_t gpio_pin_;
    adc_oneshot_unit_handle_t adc_handle_;
    int light_intensity_;  // 光照强度百分比 (0-100)
    int raw_adc_value_;    // 原始ADC值
    bool data_valid_;
    esp_timer_handle_t read_timer_;
    SemaphoreHandle_t data_mutex_;
    
    // 初始化ADC
    void InitializeAdc() {
        // 初始化ADC单元
        adc_oneshot_unit_init_cfg_t init_config = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle_));
        
        // 配置ADC通道
        adc_oneshot_chan_cfg_t chan_config = {
            .atten = ADC_ATTEN_DB_12,      // 0-3.3V量程
            .bitwidth = ADC_BITWIDTH_12,   // 12位精度
        };
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &chan_config));
        
        ESP_LOGI(TAG, "光敏传感器ADC初始化完成，GPIO引脚: %d, ADC通道: %d", gpio_pin_, LIGHT_SENSOR_ADC_CHANNEL);
    }
    
    // 读取光照强度数据
    bool ReadLightData(int& light_percent, int& raw_value) {
        esp_err_t ret = adc_oneshot_read(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC读取失败: %s", esp_err_to_name(ret));
            return false;
        }
        
        // 根据您的Arduino代码计算光照强度
        // 计算光照强度(相对比例)，并确保值在0到100之间
        light_percent = 100 - raw_value / 40;
        if (light_percent < 0) {
            light_percent = 0;
        }
        if (light_percent > 100) {
            light_percent = 100;
        }
        
        ESP_LOGD(TAG, "光照传感器读取: ADC原始值=%d, 光照强度=%d%%", raw_value, light_percent);
        return true;
    }
    
    // 定时器回调函数
    static void ReadTimerCallback(void* arg) {
        LightSensor* sensor = static_cast<LightSensor*>(arg);
        sensor->PerformReading();
    }
    
    // 执行读取操作（带重试机制）
    void PerformReading() {
        int light_percent, raw_value;
        bool success = false;
        
        // 最多重试3次
        for (int retry = 0; retry < 3 && !success; retry++) {
            if (retry > 0) {
                ESP_LOGW(TAG, "光照传感器读取失败，第%d次重试", retry);
                vTaskDelay(pdMS_TO_TICKS(50)); // 重试前等待50ms
            }
            
            success = ReadLightData(light_percent, raw_value);
        }
        
        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (success) {
                light_intensity_ = light_percent;
                raw_adc_value_ = raw_value;
                data_valid_ = true;
                ESP_LOGI(TAG, "光照传感器读取成功: 光照强度=%d%%, ADC原始值=%d", light_percent, raw_value);
            } else {
                data_valid_ = false;
                ESP_LOGW(TAG, "光照传感器读取失败，保持上次数据");
            }
            xSemaphoreGive(data_mutex_);
        }
    }

public:
    LightSensor() : Thing("LightSensor", "光敏电阻光照强度传感器，每5秒自动读取数据"), 
                    gpio_pin_(LIGHT_SENSOR_GPIO_PIN), light_intensity_(0), raw_adc_value_(0), data_valid_(false) {
        
        // 创建互斥锁
        data_mutex_ = xSemaphoreCreateMutex();
        if (data_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建数据互斥锁失败");
            return;
        }
        
        // 初始化ADC
        InitializeAdc();
        
        // 定义设备属性
        properties_.AddNumberProperty("light_intensity", "当前光照强度(%)", [this]() -> int {
            int intensity = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                intensity = data_valid_ ? light_intensity_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return intensity;
        });
        
        properties_.AddNumberProperty("raw_adc_value", "ADC原始值", [this]() -> int {
            int raw_value = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                raw_value = data_valid_ ? raw_adc_value_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return raw_value;
        });
        
        properties_.AddBooleanProperty("data_valid", "数据是否有效", [this]() -> bool {
            bool valid = false;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                valid = data_valid_;
                xSemaphoreGive(data_mutex_);
            }
            return valid;
        });
        
        // 定义设备方法
        methods_.AddMethod("read_now", "立即读取光照强度数据", ParameterList(), [this](const ParameterList& parameters) {
            PerformReading();
        });
        
        // 创建定时器，每5秒读取一次数据
        esp_timer_create_args_t timer_args = {
            .callback = ReadTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "light_read_timer",
            .skip_unhandled_events = true
        };
        
        esp_err_t ret = esp_timer_create(&timer_args, &read_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        // 启动定时器
        ret = esp_timer_start_periodic(read_timer_, LIGHT_READ_INTERVAL_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "光照传感器初始化完成，GPIO引脚: %d, 读取间隔: %dms", gpio_pin_, LIGHT_READ_INTERVAL_MS);
        
        // 立即执行一次读取
        PerformReading();
    }
    
    ~LightSensor() {
        if (read_timer_) {
            esp_timer_stop(read_timer_);
            esp_timer_delete(read_timer_);
        }
        if (adc_handle_) {
            adc_oneshot_del_unit(adc_handle_);
        }
        if (data_mutex_) {
            vSemaphoreDelete(data_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(LightSensor);
