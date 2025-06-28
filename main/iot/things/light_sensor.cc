#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#define TAG "LightSensor"

// 如果遇到ADC冲突问题，可以通过定义这个宏来禁用光照传感器
// #define DISABLE_LIGHT_SENSOR_FOR_DEBUG

// 光敏电阻 GPIO 引脚配置 (ADC)
#define LIGHT_SENSOR_GPIO_PIN GPIO_NUM_1
#define LIGHT_SENSOR_ADC_CHANNEL ADC_CHANNEL_0 // GPIO1 对应 ADC1_CH0 (ESP32-S3)

// 数据读取间隔 (5秒)
#define LIGHT_READ_INTERVAL_MS 5000

namespace iot
{
    // 全局ADC1句柄，供其他传感器共享使用
    adc_oneshot_unit_handle_t g_adc1_handle = nullptr;

    class LightSensor : public Thing
    {
    private:
        gpio_num_t gpio_pin_;
        adc_oneshot_unit_handle_t adc_handle_;
        adc_cali_handle_t adc_cali_handle_; // ADC校准句柄
        bool do_calibration_;               // 是否使用校准
        int light_intensity_;               // 光照强度百分比 (0-100)
        int raw_adc_value_;                 // 原始ADC值
        int voltage_mv_;                    // 电压值(mV)
        bool data_valid_;
        esp_timer_handle_t read_timer_;
        SemaphoreHandle_t data_mutex_;

        // 初始化ADC (基于ESP-IDF v5.0+ API，使用ADC1避免WiFi冲突)
        void InitializeAdc()
        {
            ESP_LOGI(TAG, "开始初始化ADC - GPIO%d -> ADC1_CH%d", gpio_pin_, LIGHT_SENSOR_ADC_CHANNEL);

            // ADC1初始化配置
            adc_oneshot_unit_init_cfg_t init_config1 = {
                .unit_id = ADC_UNIT_1,
            };
            esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc_handle_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "ADC单元初始化失败: %s", esp_err_to_name(ret));
                adc_handle_ = nullptr;
                return;
            }
            ESP_LOGI(TAG, "ADC1单元初始化成功");

            // 将ADC句柄赋值给全局变量，供其他传感器使用
            g_adc1_handle = adc_handle_;

            // ADC1通道配置 - 统一配置所有传感器需要的通道
            adc_oneshot_chan_cfg_t config = {
                .atten = ADC_ATTEN_DB_12,    // 11dB衰减(0-3.3V范围)
                .bitwidth = ADC_BITWIDTH_12, // 12位分辨率
            };

            // 配置光照传感器通道
            ret = adc_oneshot_config_channel(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &config);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "光照传感器ADC通道配置失败: %s", esp_err_to_name(ret));
                adc_oneshot_del_unit(adc_handle_);
                adc_handle_ = nullptr;
                return;
            }

            // 同时配置雨量传感器通道 (GPIO8 -> ADC1_CH7)
            ret = adc_oneshot_config_channel(adc_handle_, ADC_CHANNEL_7, &config);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "雨量传感器ADC通道配置失败: %s", esp_err_to_name(ret));
                adc_oneshot_del_unit(adc_handle_);
                adc_handle_ = nullptr;
                return;
            }

            ESP_LOGI(TAG, "ADC1所有通道配置完成 - 光照传感器(CH%d) + 雨量传感器(CH7)", LIGHT_SENSOR_ADC_CHANNEL);

            // 可选: 设置ADC校准
            if (do_calibration_)
            {
                adc_cali_handle_t handle = nullptr;

                // 尝试应用校准方案
                adc_cali_curve_fitting_config_t cali_config = {
                    .unit_id = ADC_UNIT_1,
                    .atten = ADC_ATTEN_DB_12,
                    .bitwidth = ADC_BITWIDTH_12,
                };

                if (adc_cali_create_scheme_curve_fitting(&cali_config, &handle) == ESP_OK)
                {
                    adc_cali_handle_ = handle;
                    ESP_LOGI(TAG, "ADC1校准已应用");
                }
                else
                {
                    ESP_LOGW(TAG, "无法使用校准方案，使用未校准值");
                    adc_cali_handle_ = nullptr;
                }
            }

            ESP_LOGI(TAG, "光敏传感器ADC初始化完成，GPIO引脚: %d, 校准: %s",
                     gpio_pin_, adc_cali_handle_ ? "已启用" : "未启用");
        }

        // 读取光照强度数据 (基于ESP-IDF v5.0+ API)
        bool ReadLightData(int &light_percent, int &raw_value, int &voltage_mv)
        {
            if (adc_handle_ == nullptr)
            {
                ESP_LOGE(TAG, "ADC句柄为空，无法读取数据");
                return false;
            }

            // 读取原始ADC值
            esp_err_t ret = adc_oneshot_read(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &raw_value);
            if (ret != ESP_OK)
            {
                ESP_LOGW(TAG, "ADC读取失败: %s (通道: %d)", esp_err_to_name(ret), LIGHT_SENSOR_ADC_CHANNEL);
                return false;
            }

            // 如果校准可用，转换为电压
            if (adc_cali_handle_)
            {
                ret = adc_cali_raw_to_voltage(adc_cali_handle_, raw_value, &voltage_mv);
                if (ret != ESP_OK)
                {
                    ESP_LOGW(TAG, "ADC校准转换失败: %s", esp_err_to_name(ret));
                    voltage_mv = 0;
                }
            }
            else
            {
                voltage_mv = 0; // 无校准时设为0
            }

            // 根据原Arduino代码计算光照强度
            // 计算光照强度(相对比例)，并确保值在0到100之间
            light_percent = 100 - raw_value / 40;
            if (light_percent < 0)
            {
                light_percent = 0;
            }
            if (light_percent > 100)
            {
                light_percent = 100;
            }

            if (adc_cali_handle_)
            {
                ESP_LOGD(TAG, "光照传感器读取: 原始值=%d, 电压=%d mV, 光照强度=%d%%",
                         raw_value, voltage_mv, light_percent);
            }
            else
            {
                ESP_LOGD(TAG, "光照传感器读取: 原始值=%d, 光照强度=%d%%",
                         raw_value, light_percent);
            }

            return true;
        }

        // 定时器回调函数
        static void ReadTimerCallback(void *arg)
        {
            LightSensor *sensor = static_cast<LightSensor *>(arg);
            sensor->PerformReading();
        }

        // 执行读取操作（带重试机制）
        void PerformReading()
        {
            int light_percent, raw_value, voltage_mv;
            bool success = false;

            // 最多重试3次
            for (int retry = 0; retry < 3 && !success; retry++)
            {
                if (retry > 0)
                {
                    ESP_LOGW(TAG, "光照传感器读取失败，第%d次重试", retry);
                    vTaskDelay(pdMS_TO_TICKS(50)); // 重试前等待50ms
                }

                success = ReadLightData(light_percent, raw_value, voltage_mv);
            }

            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE)
            {
                if (success)
                {
                    light_intensity_ = light_percent;
                    raw_adc_value_ = raw_value;
                    voltage_mv_ = voltage_mv;
                    data_valid_ = true;

                    if (adc_cali_handle_)
                    {
                        ESP_LOGI(TAG, "光照传感器读取成功: 光照强度=%d%%, ADC原始值=%d, 电压=%d mV",
                                 light_percent, raw_value, voltage_mv);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "光照传感器读取成功: 光照强度=%d%%, ADC原始值=%d",
                                 light_percent, raw_value);
                    }
                }
                else
                {
                    data_valid_ = false;
                    ESP_LOGW(TAG, "光照传感器读取失败，保持上次数据");
                }
                xSemaphoreGive(data_mutex_);
            }
        }

    public:
        LightSensor() : Thing("LightSensor", "光敏电阻光照强度传感器，每5秒自动读取数据"),
                        gpio_pin_(LIGHT_SENSOR_GPIO_PIN),
                        adc_handle_(nullptr),
                        adc_cali_handle_(nullptr),
                        do_calibration_(true),
                        light_intensity_(0),
                        raw_adc_value_(0),
                        voltage_mv_(0),
                        data_valid_(false)
        {

            ESP_LOGI(TAG, "开始初始化光照传感器...");

            // 创建互斥锁
            data_mutex_ = xSemaphoreCreateMutex();
            if (data_mutex_ == nullptr)
            {
                ESP_LOGE(TAG, "创建数据互斥锁失败");
                return;
            }

            // 延迟一段时间，等待系统稳定（特别是WiFi初始化）
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 初始化ADC
            InitializeAdc();

            // 检查ADC是否初始化成功
            if (adc_handle_ == nullptr)
            {
                ESP_LOGE(TAG, "ADC初始化失败，光照传感器将不可用");
                return;
            }

            // 定义设备属性
            properties_.AddNumberProperty("light_intensity", "当前光照强度(%)", [this]() -> int
                                          {
            int intensity = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                intensity = data_valid_ ? light_intensity_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return intensity; });

            properties_.AddNumberProperty("raw_adc_value", "ADC原始值", [this]() -> int
                                          {
            int raw_value = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                raw_value = data_valid_ ? raw_adc_value_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return raw_value; });

            properties_.AddNumberProperty("voltage_mv", "电压值(mV)", [this]() -> int
                                          {
            int voltage = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                voltage = data_valid_ ? voltage_mv_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return voltage; });

            properties_.AddBooleanProperty("calibration_enabled", "ADC校准是否启用", [this]() -> bool
                                           { return adc_cali_handle_ != nullptr; });

            properties_.AddBooleanProperty("data_valid", "数据是否有效", [this]() -> bool
                                           {
            bool valid = false;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                valid = data_valid_;
                xSemaphoreGive(data_mutex_);
            }
            return valid; });

            // 定义设备方法
            methods_.AddMethod("read_now", "立即读取光照强度数据", ParameterList(), [this](const ParameterList &parameters)
                               { PerformReading(); });

            // 添加ADC测试方法 (基于ESP-IDF v5.0+ API，使用ADC1)
            methods_.AddMethod("test_adc", "测试ADC读取", ParameterList(), [this](const ParameterList &parameters)
                               {
            ESP_LOGI(TAG, "开始ADC测试 - GPIO1(ADC1_CH0)...");
            ESP_LOGI(TAG, "校准状态: %s", adc_cali_handle_ ? "已启用" : "未启用");

            if (adc_handle_ == nullptr) {
                ESP_LOGE(TAG, "ADC句柄为空，无法进行测试");
                return;
            }

            // 连续读取10次
            for (int i = 0; i < 10; i++) {
                int raw_value = 0;
                int voltage_mv = 0;

                // 读取原始ADC值
                esp_err_t ret = adc_oneshot_read(adc_handle_, LIGHT_SENSOR_ADC_CHANNEL, &raw_value);

                if (ret == ESP_OK) {
                    // 如果校准可用，转换为电压
                    if (adc_cali_handle_) {
                        ret = adc_cali_raw_to_voltage(adc_cali_handle_, raw_value, &voltage_mv);
                        if (ret != ESP_OK) {
                            voltage_mv = 0;
                        }
                    }

                    // 计算光照强度
                    int light_percent = 100 - raw_value / 40;
                    if (light_percent < 0) light_percent = 0;
                    if (light_percent > 100) light_percent = 100;

                    if (adc_cali_handle_ && voltage_mv > 0) {
                        ESP_LOGI(TAG, "ADC测试 %d/10: 原始值=%d, 电压=%d mV, 光照强度=%d%%",
                                i+1, raw_value, voltage_mv, light_percent);
                    } else {
                        ESP_LOGI(TAG, "ADC测试 %d/10: 原始值=%d, 光照强度=%d%%",
                                i+1, raw_value, light_percent);
                    }
                } else {
                    ESP_LOGE(TAG, "ADC测试 %d/10: 读取失败 - %s", i+1, esp_err_to_name(ret));
                }

                vTaskDelay(pdMS_TO_TICKS(200)); // 200ms间隔
            }

            ESP_LOGI(TAG, "ADC测试完成"); });

            // 创建定时器，每5秒读取一次数据
            esp_timer_create_args_t timer_args = {
                .callback = ReadTimerCallback,
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "light_read_timer",
                .skip_unhandled_events = true};

            esp_err_t ret = esp_timer_create(&timer_args, &read_timer_);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
                return;
            }

            // 启动定时器
            ret = esp_timer_start_periodic(read_timer_, LIGHT_READ_INTERVAL_MS * 1000);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
                return;
            }

            ESP_LOGI(TAG, "光照传感器初始化完成，GPIO引脚: %d, 读取间隔: %dms", gpio_pin_, LIGHT_READ_INTERVAL_MS);

            // 立即执行一次读取
            PerformReading();
        }

        ~LightSensor()
        {
            // 停止并删除定时器
            if (read_timer_)
            {
                esp_timer_stop(read_timer_);
                esp_timer_delete(read_timer_);
            }

            // 释放ADC校准资源
            if (adc_cali_handle_)
            {
                adc_cali_delete_scheme_curve_fitting(adc_cali_handle_);
                adc_cali_handle_ = nullptr;
            }

            // 释放ADC单元
            if (adc_handle_)
            {
                adc_oneshot_del_unit(adc_handle_);
                adc_handle_ = nullptr;
            }

            // 删除互斥锁
            if (data_mutex_)
            {
                vSemaphoreDelete(data_mutex_);
            }

            ESP_LOGI(TAG, "光照传感器资源已释放");
        }
    };

} // namespace iot

DECLARE_THING(LightSensor);
