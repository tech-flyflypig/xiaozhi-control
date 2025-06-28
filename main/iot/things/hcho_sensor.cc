#include "iot/thing.h"
#include "board.h"

#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <string.h>

#define TAG "HCHOSensor"

// 甲醛传感器串口配置
#define HCHO_UART_NUM UART_NUM_1
#define HCHO_TXD_PIN GPIO_NUM_17
#define HCHO_RXD_PIN GPIO_NUM_18
#define HCHO_UART_BAUD_RATE 9600
#define HCHO_BUF_SIZE 256

// 数据读取间隔 (5秒)
#define HCHO_READ_INTERVAL_MS 5000

// 甲醛传感器数据包大小
#define HCHO_PACKET_SIZE 9

namespace iot {

class HCHOSensor : public Thing {
private:
    float hcho_concentration_;  // 甲醛浓度 (mg/m³)
    int raw_value_;            // 原始数值
    bool data_valid_;
    esp_timer_handle_t read_timer_;
    SemaphoreHandle_t data_mutex_;
    uint8_t uart_buffer_[HCHO_BUF_SIZE];
    uint8_t rx_data_[HCHO_PACKET_SIZE];  // 接收数据缓冲区
    uint8_t rx_state_;                   // 接收状态机状态
    uint8_t rx_packet_index_;            // 当前接收字节索引
    
    // 初始化UART
    void InitializeUart() {
        uart_config_t uart_config = {
            .baud_rate = HCHO_UART_BAUD_RATE,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
        };
        
        // 配置UART参数
        ESP_ERROR_CHECK(uart_param_config(HCHO_UART_NUM, &uart_config));
        
        // 设置UART引脚
        ESP_ERROR_CHECK(uart_set_pin(HCHO_UART_NUM, HCHO_TXD_PIN, HCHO_RXD_PIN, 
                                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        
        // 安装UART驱动
        ESP_ERROR_CHECK(uart_driver_install(HCHO_UART_NUM, HCHO_BUF_SIZE * 2, 0, 0, NULL, 0));
        
        ESP_LOGI(TAG, "甲醛传感器UART初始化完成，波特率: %d, TXD: %d, RXD: %d", 
                 HCHO_UART_BAUD_RATE, HCHO_TXD_PIN, HCHO_RXD_PIN);
    }
    
    // 计算校验和（根据您的Arduino代码实现）
    uint8_t CalculateChecksum(uint8_t* data, uint8_t len) {
        uint8_t tempq = 0;
        data += 1;  // 跳过第一个字节
        for (uint8_t j = 0; j < (len - 2); j++) {
            tempq += *data;
            data++;
        }
        tempq = (~tempq) + 1;
        return tempq;
    }
    
    // 处理接收到的数据包（根据您的Arduino代码实现）
    int32_t DealWithData(uint8_t* data, float& concentration, int& raw_val) {
        if (data[0] == 0xFF && data[1] == 0x17) {
            if (CalculateChecksum(data, HCHO_PACKET_SIZE) == data[8]) {
                raw_val = (data[4] << 8) + data[5];
                concentration = raw_val;  // 直接使用原始值作为μg/m³
                ESP_LOGD(TAG, "甲醛传感器解析成功: 原始值=%d, 浓度=%.0f μg/m³", raw_val, concentration);
                return 0;  // 数据处理成功
            }
            ESP_LOGW(TAG, "甲醛传感器校验失败");
            return -1;  // 校验失败
        }
        ESP_LOGW(TAG, "甲醛传感器数据格式不匹配");
        return -2;  // 数据格式不匹配
    }
    
    // 读取甲醛数据（使用状态机接收，根据您的Arduino代码实现）
    bool ReadHCHOData(float& concentration, int& raw_val) {
        // 读取串口数据并处理
        int len = uart_read_bytes(HCHO_UART_NUM, uart_buffer_, HCHO_BUF_SIZE - 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            return false;  // 没有数据可读
        }

        // 处理接收到的每个字节
        for (int i = 0; i < len; i++) {
            uint8_t indata = uart_buffer_[i];

            switch (rx_state_) {
                case 0:  // 等待数据包起始字节
                    if (indata == 0xFF) {
                        rx_state_ = 1;
                        rx_packet_index_ = 0;
                        rx_data_[rx_packet_index_++] = indata;
                    }
                    break;

                case 1:  // 接收数据包中的数据字节
                    if (rx_packet_index_ < HCHO_PACKET_SIZE) {
                        rx_data_[rx_packet_index_++] = indata;
                        if (rx_packet_index_ >= HCHO_PACKET_SIZE) {
                            int32_t ret = DealWithData(rx_data_, concentration, raw_val);
                            rx_state_ = 0;
                            return (ret == 0);  // 返回是否成功
                        }
                    } else {
                        rx_state_ = 0;
                        return false;  // 数据包过长
                    }
                    break;

                default:
                    rx_state_ = 0;
                    return false;  // 未知错误
            }
        }

        return false;  // 未收到完整数据包
    }
    
    // 定时器回调函数
    static void ReadTimerCallback(void* arg) {
        HCHOSensor* sensor = static_cast<HCHOSensor*>(arg);
        sensor->PerformReading();
    }
    
    // 执行读取操作（带重试机制）
    void PerformReading() {
        float concentration;
        int raw_val;
        bool success = false;
        
        // 最多重试3次
        for (int retry = 0; retry < 3 && !success; retry++) {
            if (retry > 0) {
                ESP_LOGW(TAG, "甲醛传感器读取失败，第%d次重试", retry);
                vTaskDelay(pdMS_TO_TICKS(100)); // 重试前等待100ms
            }
            
            success = ReadHCHOData(concentration, raw_val);
        }
        
        if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (success) {
                hcho_concentration_ = concentration;
                raw_value_ = raw_val;
                data_valid_ = true;
                ESP_LOGI(TAG, "甲醛传感器读取成功: 浓度=%.0f μg/m³, 原始值=%d", concentration, raw_val);
            } else {
                data_valid_ = false;
                ESP_LOGW(TAG, "甲醛传感器读取失败，保持上次数据");
            }
            xSemaphoreGive(data_mutex_);
        }
    }

public:
    HCHOSensor() : Thing("HCHOSensor", "甲醛传感器，每5秒自动读取数据"),
                   hcho_concentration_(0.0f), raw_value_(0), data_valid_(false),
                   rx_state_(0), rx_packet_index_(0) {
        
        // 创建互斥锁
        data_mutex_ = xSemaphoreCreateMutex();
        if (data_mutex_ == nullptr) {
            ESP_LOGE(TAG, "创建数据互斥锁失败");
            return;
        }
        
        // 初始化UART
        InitializeUart();
        
        // 定义设备属性
        properties_.AddNumberProperty("hcho_concentration", "甲醛浓度(μg/m³)", [this]() -> float {
            float concentration = 0.0f;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                concentration = data_valid_ ? hcho_concentration_ : 0.0f;
                xSemaphoreGive(data_mutex_);
            }
            return concentration;
        });

        // 添加整数版本的甲醛浓度，方便AI读取
        properties_.AddNumberProperty("hcho_level", "甲醛浓度等级(μg/m³)", [this]() -> int {
            int level = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                level = data_valid_ ? (int)hcho_concentration_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return level;
        });
        
        properties_.AddNumberProperty("raw_value", "原始数值", [this]() -> int {
            int raw_val = 0;
            if (xSemaphoreTake(data_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
                raw_val = data_valid_ ? raw_value_ : 0;
                xSemaphoreGive(data_mutex_);
            }
            return raw_val;
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
        methods_.AddMethod("read_now", "立即读取甲醛浓度数据", ParameterList(), [this](const ParameterList& parameters) {
            PerformReading();
        });
        
        // 创建定时器，每5秒读取一次数据
        esp_timer_create_args_t timer_args = {
            .callback = ReadTimerCallback,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "hcho_read_timer",
            .skip_unhandled_events = true
        };
        
        esp_err_t ret = esp_timer_create(&timer_args, &read_timer_);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        // 启动定时器
        ret = esp_timer_start_periodic(read_timer_, HCHO_READ_INTERVAL_MS * 1000);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(ret));
            return;
        }
        
        ESP_LOGI(TAG, "甲醛传感器初始化完成，读取间隔: %dms", HCHO_READ_INTERVAL_MS);
        
        // 立即执行一次读取
        PerformReading();
    }
    
    ~HCHOSensor() {
        if (read_timer_) {
            esp_timer_stop(read_timer_);
            esp_timer_delete(read_timer_);
        }
        uart_driver_delete(HCHO_UART_NUM);
        if (data_mutex_) {
            vSemaphoreDelete(data_mutex_);
        }
    }
};

} // namespace iot

DECLARE_THING(HCHOSensor);
