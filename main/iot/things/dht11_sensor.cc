#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <rom/ets_sys.h>

#define TAG "DHT11Sensor"

// DHT11 GPIO 引脚配置
#ifdef CONFIG_IDF_TARGET_ESP32
#define DHT11_GPIO_PIN GPIO_NUM_12
#else
#define DHT11_GPIO_PIN GPIO_NUM_12
#endif

// DHT11 时序定义 (微秒)
#define DHT11_START_SIGNAL_LOW_TIME 20000   // 20ms (增加到20ms确保稳定)
#define DHT11_START_SIGNAL_HIGH_TIME 30     // 30us
#define DHT11_RESPONSE_TIMEOUT_US 100       // 100us 响应超时
#define DHT11_DATA_TIMEOUT_US 200           // 200us 数据位超时
#define DHT11_BIT_THRESHOLD_US 50           // 50us 作为0/1判断阈值

// 数据读取间隔 (5秒)
#define DHT11_READ_INTERVAL_MS 5000

namespace iot {

class DHT11Sensor : public Thing {
private:
    gpio_num_t gpio_pin_;
    float temperature_;
    float humidity_;
    bool data_valid_;
    esp_timer_handle_t read_timer_;
    SemaphoreHandle_t data_mutex_;
    
    // DHT11 驱动相关方法
    void InitializeGpio() {
        // 初始化为输出模式，默认高电平
        gpio_reset_pin(gpio_pin_);
        gpio_set_direction(gpio_pin_, GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(gpio_pin_, GPIO_PULLUP_ONLY);
        gpio_set_level(gpio_pin_, 1);
        ESP_LOGI(TAG, "DHT11 GPIO %d 初始化完成", gpio_pin_);
    }

    void SetGpioOutput() {
        gpio_set_direction(gpio_pin_, GPIO_MODE_OUTPUT);
    }

    void SetGpioInput() {
        gpio_set_direction(gpio_pin_, GPIO_MODE_INPUT);
        gpio_set_pull_mode(gpio_pin_, GPIO_PULLUP_ONLY);
    }
    
    // 等待GPIO电平变化，带超时
    bool WaitForLevel(int level, int timeout_us) {
        int elapsed = 0;
        while (gpio_get_level(gpio_pin_) != level) {
            ets_delay_us(2);  // 增加延时精度
            elapsed += 2;
            if (elapsed > timeout_us) {
                return false;
            }
        }
        return true;
    }

    // 测量高电平持续时间
    int MeasureHighTime(int timeout_us) {
        int duration = 0;
        while (gpio_get_level(gpio_pin_) == 1 && duration < timeout_us) {
            ets_delay_us(2);
            duration += 2;
        }
        return duration;
    }
    
    // 读取DHT11数据
    bool ReadDHT11Data(float& temp, float& hum) {
        uint8_t data[5] = {0};

        // 禁用中断，确保时序准确
        portDISABLE_INTERRUPTS();

        // 发送开始信号
        SetGpioOutput();
        gpio_set_level(gpio_pin_, 0);
        ets_delay_us(DHT11_START_SIGNAL_LOW_TIME);  // 20ms低电平
        gpio_set_level(gpio_pin_, 1);
        ets_delay_us(DHT11_START_SIGNAL_HIGH_TIME); // 30us高电平

        // 切换到输入模式
        SetGpioInput();

        // 等待DHT11响应信号
        // DHT11会先拉低80us，然后拉高80us
        if (!WaitForLevel(0, DHT11_RESPONSE_TIMEOUT_US)) {
            portENABLE_INTERRUPTS();
            ESP_LOGW(TAG, "DHT11响应超时 - 等待低电平");
            return false;
        }

        if (!WaitForLevel(1, DHT11_RESPONSE_TIMEOUT_US)) {
            portENABLE_INTERRUPTS();
            ESP_LOGW(TAG, "DHT11响应超时 - 等待高电平");
            return false;
        }

        if (!WaitForLevel(0, DHT11_RESPONSE_TIMEOUT_US)) {
            portENABLE_INTERRUPTS();
            ESP_LOGW(TAG, "DHT11响应超时 - 等待数据开始");
            return false;
        }
        
        // 读取40位数据
        for (int i = 0; i < 40; i++) {
            // 等待数据位开始 (每个数据位都以50us低电平开始)
            if (!WaitForLevel(1, DHT11_DATA_TIMEOUT_US)) {
                portENABLE_INTERRUPTS();
                ESP_LOGW(TAG, "DHT11数据位%d超时 - 等待高电平", i);
                return false;
            }

            // 测量高电平持续时间
            // 数据0: 26-28us高电平
            // 数据1: 70us高电平
            int high_time = MeasureHighTime(DHT11_DATA_TIMEOUT_US);
            if (high_time >= DHT11_DATA_TIMEOUT_US) {
                portENABLE_INTERRUPTS();
                ESP_LOGW(TAG, "DHT11数据位%d超时 - 高电平持续时间", i);
                return false;
            }

            // 根据高电平持续时间判断是0还是1
            int byte_index = i / 8;
            int bit_index = 7 - (i % 8);

            if (high_time > DHT11_BIT_THRESHOLD_US) {  // 大于50us认为是1
                data[byte_index] |= (1 << bit_index);
            }
            // 小于等于50us认为是0，不需要额外操作
        }

        // 恢复中断
        portENABLE_INTERRUPTS();
        
        // 校验数据
        uint8_t checksum = data[0] + data[1] + data[2] + data[3];
        if (checksum != data[4]) {
            ESP_LOGW(TAG, "DHT11校验失败: 计算=%d, 接收=%d", checksum, data[4]);
            return false;
        }
        
        // 解析数据
        hum = data[0] + data[1] * 0.1f;
        temp = data[2] + data[3] * 0.1f;

        return true;
    }
    
    // 定时器回调函数
    static void ReadTimerCallback(void* arg) {
        DHT11Sensor* sensor = static_cast<DHT11Sensor*>(arg);
        sensor->PerformReading();
    }
    
    // 执行读取操作（带重试机制）
    void PerformReading() {
        float temp, hum;
        bool success = false;

        // 最多重试3次
        for (int retry = 0; retry < 3 && !success; retry++) {
            if (retry > 0) {
                ESP_LOGW(TAG, "DHT11读取失败，第%d次重试", retry);
                vTaskDelay(pdMS_TO_TICKS(100)); // 重试前等待100ms
            }

            success = ReadDHT11Data(temp, hum);
        }

        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (success) {
                temperature_ = temp;
                humidity_ = hum;
                data_valid_ = true;
                ESP_LOGI(TAG, "DHT11读取成功: 温度=%.1f°C, 湿度=%.1f%%", temp, hum);
            } else {
                data_valid_ = false;
                ESP_LOGW(TAG, "DHT11读取失败，保持上次数据");
            }
            xSemaphoreGive(data_mutex_);
        }
    }

public:
    DHT11Sensor() : Thing("DHT11Sensor", "DHT11温湿度传感器，每5秒自动读取数据"), 
                    gpio_pin_(DHT11_GPIO_PIN), temperature_(0.0f), humidity_(0.0f), data_valid_(false) {
        
        // 创建互斥锁
        data_mutex_ = xSemaphoreCreateMutex();
        if (data_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建数据互斥锁失败");
            return;
        }
        
        // 初始化GPIO
        InitializeGpio();
        
        // 定义设备属性
        properties_.AddNumberProperty("temperature", "当前温度(°C)", [this]() -> float {
            float temp = 0.0f;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                temp = data_valid_ ? temperature_ : 0.0f;
                xSemaphoreGive(data_mutex_);
            }
            return temp;
        });
        
        properties_.AddNumberProperty("humidity", "当前湿度(%)", [this]() -> float {
            float hum = 0.0f;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                hum = data_valid_ ? humidity_ : 0.0f;
                xSemaphoreGive(data_mutex_);
            }
            return hum;
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
        methods_.AddMethod("read_now", "立即读取温湿度数据", ParameterList(), [this](const ParameterList& parameters) {
            PerformReading();
        });
        
        // 创建定时器，每5秒读取一次数据
        esp_timer_create_args_t timer_args = {
            .callback = ReadTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dht11_read_timer",
            .skip_unhandled_events = true
        };
        
        esp_err_t ret = esp_timer_create(&timer_args, &read_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        // 启动定时器
        ret = esp_timer_start_periodic(read_timer_, DHT11_READ_INTERVAL_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "DHT11传感器初始化完成，GPIO引脚: %d, 读取间隔: %dms", gpio_pin_, DHT11_READ_INTERVAL_MS);
        
        // 立即执行一次读取
        PerformReading();
    }
    
    ~DHT11Sensor() {
        if (read_timer_) {
            esp_timer_stop(read_timer_);
            esp_timer_delete(read_timer_);
        }
        if (data_mutex_) {
            vSemaphoreDelete(data_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(DHT11Sensor);
