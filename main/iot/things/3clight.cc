#include "iot/thing.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char* TAG = "3CLight";

namespace iot {

class ThreeColorLight : public Thing {
private:
    gpio_num_t red_gpio_;
    gpio_num_t green_gpio_;
    gpio_num_t blue_gpio_;
    
    bool red_state_ = false;
    bool green_state_ = false;
    bool blue_state_ = false;

    void InitializeGpio() {
        // 配置红色LED GPIO
        gpio_config_t red_config = {
            .pin_bit_mask = (1ULL << red_gpio_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&red_config);
        
        // 配置绿色LED GPIO
        gpio_config_t green_config = {
            .pin_bit_mask = (1ULL << green_gpio_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&green_config);
        
        // 配置蓝色LED GPIO
        gpio_config_t blue_config = {
            .pin_bit_mask = (1ULL << blue_gpio_),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&blue_config);
        
        // 初始状态设为关闭
        UpdateGpioState();
        
        ESP_LOGI(TAG, "三色灯GPIO初始化完成 - 红:%d, 绿:%d, 蓝:%d", 
                 red_gpio_, green_gpio_, blue_gpio_);
    }

    void UpdateGpioState() {
        gpio_set_level(red_gpio_, red_state_ ? 1 : 0);
        gpio_set_level(green_gpio_, green_state_ ? 1 : 0);
        gpio_set_level(blue_gpio_, blue_state_ ? 1 : 0);
    }

public:
    // 构造函数，可配置GPIO引脚
    ThreeColorLight(gpio_num_t red_gpio = GPIO_NUM_3, 
                   gpio_num_t green_gpio = GPIO_NUM_46, 
                   gpio_num_t blue_gpio = GPIO_NUM_9) 
        : Thing("3clight", "三色LED灯，支持红绿蓝三种颜色的独立控制"),
          red_gpio_(red_gpio),
          green_gpio_(green_gpio),
          blue_gpio_(blue_gpio) {
        
        // 初始化GPIO
        InitializeGpio();
        
        // 定义属性：三种颜色的开关状态
        properties_.AddBooleanProperty("red", "红色LED是否打开", [this]() -> bool {
            return red_state_;
        });
        
        properties_.AddBooleanProperty("green", "绿色LED是否打开", [this]() -> bool {
            return green_state_;
        });
        
        properties_.AddBooleanProperty("blue", "蓝色LED是否打开", [this]() -> bool {
            return blue_state_;
        });
        
        // 定义组合属性：当前点亮的颜色组合
        properties_.AddStringProperty("color_combination", "当前点亮的颜色组合", [this]() -> std::string {
            std::string result = "";
            if (red_state_) result += "红";
            if (green_state_) result += "绿";  
            if (blue_state_) result += "蓝";
            return result.empty() ? "关闭" : result;
        });
        
        // 定义方法：单独控制每种颜色
        methods_.AddMethod("TurnOnRed", "打开红色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            red_state_ = true;
            UpdateGpioState();
            ESP_LOGI(TAG, "红色LED已打开");
        });
        
        methods_.AddMethod("TurnOffRed", "关闭红色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            red_state_ = false;
            UpdateGpioState();
            ESP_LOGI(TAG, "红色LED已关闭");
        });
        
        methods_.AddMethod("TurnOnGreen", "打开绿色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            green_state_ = true;
            UpdateGpioState();
            ESP_LOGI(TAG, "绿色LED已打开");
        });
        
        methods_.AddMethod("TurnOffGreen", "关闭绿色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            green_state_ = false;
            UpdateGpioState();
            ESP_LOGI(TAG, "绿色LED已关闭");
        });
        
        methods_.AddMethod("TurnOnBlue", "打开蓝色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            blue_state_ = true;
            UpdateGpioState();
            ESP_LOGI(TAG, "蓝色LED已打开");
        });
        
        methods_.AddMethod("TurnOffBlue", "关闭蓝色LED", ParameterList(), 
                          [this](const ParameterList& parameters) {
            blue_state_ = false;
            UpdateGpioState();
            ESP_LOGI(TAG, "蓝色LED已关闭");
        });
        
        // 定义方法：颜色组合控制
        methods_.AddMethod("SetColor", "设置指定颜色组合", ParameterList({
            Parameter("color", "颜色名称：红/绿/蓝/黄(红+绿)/紫(红+蓝)/青(绿+蓝)/白(红+绿+蓝)/关闭", kValueTypeString, true)
        }), [this](const ParameterList& parameters) {
            std::string color = parameters["color"].string();
            
            // 先全部关闭
            red_state_ = false;
            green_state_ = false;
            blue_state_ = false;
            
            // 根据颜色名称设置对应状态
            if (color == "红" || color == "红色") {
                red_state_ = true;
            } else if (color == "绿" || color == "绿色") {
                green_state_ = true;
            } else if (color == "蓝" || color == "蓝色") {
                blue_state_ = true;
            } else if (color == "黄" || color == "黄色") {
                red_state_ = true;
                green_state_ = true;
            } else if (color == "紫" || color == "紫色") {
                red_state_ = true;
                blue_state_ = true;
            } else if (color == "青" || color == "青色") {
                green_state_ = true;
                blue_state_ = true;
            } else if (color == "白" || color == "白色") {
                red_state_ = true;
                green_state_ = true;
                blue_state_ = true;
            } else if (color == "关闭" || color == "关") {
                // 已经全部设为false
            }
            
            UpdateGpioState();
            ESP_LOGI(TAG, "颜色设置为: %s", color.c_str());
        });
        
        // 定义方法：全部开关控制
        methods_.AddMethod("TurnOnAll", "打开所有颜色(白光)", ParameterList(), 
                          [this](const ParameterList& parameters) {
            red_state_ = true;
            green_state_ = true;
            blue_state_ = true;
            UpdateGpioState();
            ESP_LOGI(TAG, "所有颜色已打开(白光)");
        });
        
        methods_.AddMethod("TurnOffAll", "关闭所有颜色", ParameterList(), 
                          [this](const ParameterList& parameters) {
            red_state_ = false;
            green_state_ = false;
            blue_state_ = false;
            UpdateGpioState();
            ESP_LOGI(TAG, "所有颜色已关闭");
        });
        
        // 定义方法：切换单个颜色状态
        methods_.AddMethod("ToggleColor", "切换指定颜色的开关状态", ParameterList({
            Parameter("color", "要切换的颜色：红/绿/蓝", kValueTypeString, true)
        }), [this](const ParameterList& parameters) {
            std::string color = parameters["color"].string();
            
            if (color == "红" || color == "红色") {
                red_state_ = !red_state_;
                ESP_LOGI(TAG, "红色LED切换为: %s", red_state_ ? "开" : "关");
            } else if (color == "绿" || color == "绿色") {
                green_state_ = !green_state_;
                ESP_LOGI(TAG, "绿色LED切换为: %s", green_state_ ? "开" : "关");
            } else if (color == "蓝" || color == "蓝色") {
                blue_state_ = !blue_state_;
                ESP_LOGI(TAG, "蓝色LED切换为: %s", blue_state_ ? "开" : "关");
            }
            
            UpdateGpioState();
        });
    }
    
    ~ThreeColorLight() = default;
};

} // namespace iot
// 注册设备到系统
DECLARE_THING(ThreeColorLight);