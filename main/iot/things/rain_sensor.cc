#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "RainSensor"

// 雨量传感器 GPIO 引脚配置 (ADC)
#define RAIN_SENSOR_GPIO_PIN GPIO_NUM_8
#define RAIN_SENSOR_ADC_CHANNEL ADC_CHANNEL_7  // GPIO8 对应 ADC1_CHANNEL_7

// 数据读取间隔 (5秒)
#define RAIN_READ_INTERVAL_MS 5000

namespace iot {

// 声明外部ADC句柄（由光照传感器初始化）
extern adc_oneshot_unit_handle_t g_adc1_handle;

class RainSensor : public Thing {
private:
    gpio_num_t gpio_pin_;
    int rain_intensity_;   // 雨量强度百分比 (0-100)
    int raw_adc_value_;    // 原始ADC值
    bool data_valid_;
    esp_timer_handle_t read_timer_;
    SemaphoreHandle_t data_mutex_;

    // 检查ADC是否可用（ADC1单元和通道都已由光照传感器初始化）
    void CheckAdcAvailability() {
        if (g_adc1_handle == nullptr) {
            ESP_LOGE(TAG, "ADC1句柄未初始化，请确保光照传感器先初始化");
            return;
        }

        ESP_LOGI(TAG, "雨量传感器使用共享ADC1，GPIO引脚: %d, ADC通道: %d", gpio_pin_, RAIN_SENSOR_ADC_CHANNEL);
    }
    
    // 读取雨量强度数据
    bool ReadRainData(int& rain_percent, int& raw_value) {
        esp_err_t ret = adc_oneshot_read(g_adc1_handle, RAIN_SENSOR_ADC_CHANNEL, &raw_value);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "ADC读取失败: %s", esp_err_to_name(ret));
            return false;
        }
        
        // 根据您的Arduino代码计算雨量强度
        // rainValue = 100 - analogRead(Sensor_Rain_A_PIN) / 4095.00 * 100;
        rain_percent = 100 - (raw_value * 100) / 4095;
        if (rain_percent < 0) {
            rain_percent = 0;
        }
        if (rain_percent > 100) {
            rain_percent = 100;
        }
        
        ESP_LOGD(TAG, "雨量传感器读取: ADC原始值=%d, 雨量强度=%d%%", raw_value, rain_percent);
        return true;
    }
    
    // 定时器回调函数
    static void ReadTimerCallback(void* arg) {
        RainSensor* sensor = static_cast<RainSensor*>(arg);
        sensor->PerformReading();
    }
    
    // 执行读取操作（带重试机制）
    void PerformReading() {
        int rain_percent, raw_value;
        bool success = false;
        
        // 最多重试3次
        for (int retry = 0; retry < 3 && !success; retry++) {
            if (retry > 0) {
                ESP_LOGW(TAG, "雨量传感器读取失败，第%d次重试", retry);
                vTaskDelay(pdMS_TO_TICKS(50)); // 重试前等待50ms
            }
            
            success = ReadRainData(rain_percent, raw_value);
        }
        
        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (success) {
                rain_intensity_ = rain_percent;
                raw_adc_value_ = raw_value;
                data_valid_ = true;
                ESP_LOGI(TAG, "雨量传感器读取成功: 雨量强度=%d%%, ADC原始值=%d", rain_percent, raw_value);
            } else {
                data_valid_ = false;
                ESP_LOGW(TAG, "雨量传感器读取失败，保持上次数据");
            }
            xSemaphoreGive(data_mutex_);
        }
    }

public:
    RainSensor() : Thing("RainSensor", "雨量传感器，每5秒自动读取数据"), 
                   gpio_pin_(RAIN_SENSOR_GPIO_PIN), rain_intensity_(0), raw_adc_value_(0), data_valid_(false) {
        
        // 创建互斥锁
        data_mutex_ = xSemaphoreCreateMutex();
        if (data_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建数据互斥锁失败");
            return;
        }
        
        // 检查ADC可用性（ADC1和通道都已由光照传感器初始化）
        CheckAdcAvailability();
        
        // 定义设备属性
        properties_.AddNumberProperty("rain_intensity", "当前雨量强度(%)", [this]() -> int {
            int intensity = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                intensity = data_valid_ ? rain_intensity_ : 0;
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
        methods_.AddMethod("read_now", "立即读取雨量强度数据", ParameterList(), [this](const ParameterList& parameters) {
            PerformReading();
        });
        
        // 创建定时器，每5秒读取一次数据
        esp_timer_create_args_t timer_args = {
            .callback = ReadTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "rain_read_timer",
            .skip_unhandled_events = true
        };
        
        esp_err_t ret = esp_timer_create(&timer_args, &read_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        // 启动定时器
        ret = esp_timer_start_periodic(read_timer_, RAIN_READ_INTERVAL_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "雨量传感器初始化完成，GPIO引脚: %d, 读取间隔: %dms", gpio_pin_, RAIN_READ_INTERVAL_MS);
        
        // 立即执行一次读取
        PerformReading();
    }
    
    ~RainSensor() {
        if (read_timer_) {
            esp_timer_stop(read_timer_);
            esp_timer_delete(read_timer_);
        }
        // 注意：不删除ADC单元，因为它由光照传感器管理
        if (data_mutex_) {
            vSemaphoreDelete(data_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(RainSensor);
