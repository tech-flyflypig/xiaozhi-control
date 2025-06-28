#include "iot/thing.h"
#include "iot/thing_manager.h"
#include "board.h"
#include "application.h"
#include "esp_log.h"
#include <stdexcept> // 添加异常处理头文件
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gptimer.h> // 添加硬件定时器头文件

static const char *TAG = "WindowController";

// 定义ULN2003驱动的步进电机控制引脚
// 注意：避免与现有功能冲突，选择未被占用的GPIO引脚，如有冲突，请修改为未被占用的GPIO引脚
// 修改为GPIO36-39，避免与UART1(GPIO17/18)冲突
#define STEPPER_IN1_PIN GPIO_NUM_19  // ULN2003 IN1
#define STEPPER_IN2_PIN GPIO_NUM_20 // ULN2003 IN2
#define STEPPER_IN3_PIN GPIO_NUM_10 // ULN2003 IN3
#define STEPPER_IN4_PIN GPIO_NUM_11 // ULN2003 IN4

// 定义步进电机参数
#define STEPPER_FULL_REVOLUTION_STEPS 2048 // 28BYJ-48步进电机一圈步数(通常是2048步/圈)
#define WINDOW_FULL_STEPS 2048             // 窗户开关所需步数
#define DEFAULT_STEP_DELAY_MS 5           // 默认步进延迟(ms)，28BYJ-48需要较长延迟

namespace iot
{

    class WindowController : public Thing
    {
    private:
        // 步进电机控制相关
        gpio_num_t in1_pin_; // IN1引脚
        gpio_num_t in2_pin_; // IN2引脚
        gpio_num_t in3_pin_; // IN3引脚
        gpio_num_t in4_pin_; // IN4引脚

        // 步进序列 - 使用4步序列模式，更适合28BYJ-48步进电机
        static const int numSteps = 4;
        const uint8_t stepSequence[4][4] = {
            {1, 0, 0, 1}, // Step 1: IN1=1, IN2=0, IN3=0, IN4=1
            {1, 1, 0, 0}, // Step 2: IN1=1, IN2=1, IN3=0, IN4=0
            {0, 1, 1, 0}, // Step 3: IN1=0, IN2=1, IN3=1, IN4=0
            {0, 0, 1, 1}  // Step 4: IN1=0, IN2=0, IN3=1, IN4=1
        };

        int current_step_ = 0;                      // 当前步进序列索引
        bool is_running_ = false;                   // 电机是否正在运行
        bool direction_ = true;                     // 电机方向(true为正向，用于打开窗户)
        int position_ = 0;                          // 当前位置(步数)
        int target_position_ = 0;                   // 目标位置(步数)
        int step_delay_ms_ = DEFAULT_STEP_DELAY_MS; // 步进延迟时间(ms)
        bool window_opened_ = false;                // 窗户状态(false:关闭, true:打开)

        // 硬件定时器相关
        gptimer_handle_t gptimer = nullptr;                     // 定时器句柄
        portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED; // 自旋锁，用于保护步进电机操作

        // 中断标志，用于通知主任务处理步进
        volatile bool step_requested_ = false;
        SemaphoreHandle_t step_semaphore_ = nullptr;

        // 初始化GPIO
        void InitializeGpio()
        {
            // 配置四个控制引脚
            gpio_config_t io_conf = {};
            io_conf.mode = GPIO_MODE_OUTPUT;
            io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
            io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
            io_conf.intr_type = GPIO_INTR_DISABLE;

            // 配置所有引脚
            io_conf.pin_bit_mask = (1ULL << in1_pin_) | (1ULL << in2_pin_) | (1ULL << in3_pin_) | (1ULL << in4_pin_);
            gpio_config(&io_conf);

            // 初始状态：所有引脚低电平
            StopMotor();

            ESP_LOGI(TAG, "ULN2003步进电机GPIO初始化完成 - IN1:%d, IN2:%d, IN3:%d, IN4:%d",
                     in1_pin_, in2_pin_, in3_pin_, in4_pin_);

            // 初始化硬件定时器
            InitializeTimer();

            // 运行简单的电机测试
            TestMotor();
        }

        // 初始化硬件定时器
        void InitializeTimer()
        {
            ESP_LOGI(TAG, "初始化硬件定时器开始");

            // 创建信号量用于步进通知
            step_semaphore_ = xSemaphoreCreateBinary();
            if (step_semaphore_ == nullptr)
            {
                ESP_LOGE(TAG, "无法创建步进信号量");
                return;
            }
            ESP_LOGI(TAG, "步进信号量创建成功");

            // 配置定时器 - 使用更安全的初始化方式
            gptimer_config_t timer_config = {
                .clk_src = GPTIMER_CLK_SRC_DEFAULT,
                .direction = GPTIMER_COUNT_UP,
                .resolution_hz = 1000000, // 1MHz, 1 tick = 1us
                .intr_priority = 0,       // 使用默认优先级
                .flags = {
                    .intr_shared = 0,
                    .allow_pd = 0}};

            ESP_LOGI(TAG, "定时器配置已准备，准备创建定时器");

            // 使用更严格的错误检查
            esp_err_t err = gptimer_new_timer(&timer_config, &gptimer);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "创建定时器失败: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "定时器创建成功");

            // 配置回调，使用更安全的初始化方式
            gptimer_event_callbacks_t cbs = {
                .on_alarm = StepperTimerCallback};

            ESP_LOGI(TAG, "准备注册定时器回调函数");
            err = gptimer_register_event_callbacks(gptimer, &cbs, this);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "注册定时器回调函数失败: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "定时器回调函数注册成功");

            // 使能定时器
            err = gptimer_enable(gptimer);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "使能定时器失败: %s", esp_err_to_name(err));
                return;
            }
            ESP_LOGI(TAG, "定时器启用成功");

            // 创建步进处理任务
            BaseType_t task_created = xTaskCreate(StepperTaskFunc, "stepper_task", 4096, this, 5, nullptr);
            if (task_created != pdPASS)
            {
                ESP_LOGE(TAG, "创建步进处理任务失败");
                return;
            }
            ESP_LOGI(TAG, "步进处理任务创建成功");

            ESP_LOGI(TAG, "硬件定时器初始化完成");
        }

        // 定时器回调函数 - 极简版本，只发送信号量
        static bool IRAM_ATTR StepperTimerCallback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
        {
            // 检查参数有效性
            if (!user_data || !timer)
            {
                return false;
            }

            WindowController *controller = static_cast<WindowController *>(user_data);
            BaseType_t high_task_awoken = pdFALSE;

            // 安全地访问信号量
            SemaphoreHandle_t semaphore = controller->step_semaphore_;
            if (semaphore)
            {
                // 只设置标志并通知任务
                controller->step_requested_ = true;
                xSemaphoreGiveFromISR(semaphore, &high_task_awoken);
            }

            return high_task_awoken == pdTRUE;
        }

        // 步进电机处理任务函数
        static void StepperTaskFunc(void *arg)
        {
            WindowController *controller = static_cast<WindowController *>(arg);
            controller->RunStepperTask();
            vTaskDelete(nullptr);
        }

        // 步进电机处理任务
        void RunStepperTask()
        {
            ESP_LOGI(TAG, "步进电机处理任务开始执行");

            // 检查信号量是否已创建
            if (!step_semaphore_)
            {
                ESP_LOGE(TAG, "步进信号量未初始化，步进处理任务退出");
                return;
            }

            // 任务主循环
            while (true)
            {
                // 等待定时器中断通知
                if (xSemaphoreTake(step_semaphore_, pdMS_TO_TICKS(1000)) == pdTRUE)
                {
                    // 重置请求标志
                    step_requested_ = false;

                    // 如果电机不在运行状态，忽略
                    if (!is_running_)
                    {
                        continue;
                    }

                    // 如果已经到达目标位置，停止电机
                    if (position_ == target_position_)
                    {
                        is_running_ = false;
                        if (gptimer)
                        {
                            esp_err_t err = gptimer_stop(gptimer);
                            if (err != ESP_OK)
                            {
                                ESP_LOGW(TAG, "停止定时器失败: %s", esp_err_to_name(err));
                            }
                        }
                        StopMotor();
                        continue;
                    }

                    // 计算方向
                    int dir = (target_position_ > position_) ? 1 : -1;

                    try
                    {
                        // 执行步进
                        portENTER_CRITICAL(&stepperMux);

                        // 更新步进序列索引
                        current_step_ = ((current_step_ + dir) + numSteps) % numSteps;

                        // 输出步进序列
                        gpio_set_level(in1_pin_, stepSequence[current_step_][0]);
                        gpio_set_level(in2_pin_, stepSequence[current_step_][1]);
                        gpio_set_level(in3_pin_, stepSequence[current_step_][2]);
                        gpio_set_level(in4_pin_, stepSequence[current_step_][3]);

                        portEXIT_CRITICAL(&stepperMux);

                        // 更新位置
                        position_ += dir;
                    }
                    catch (...)
                    {
                        // 捕获任何异常，确保不会导致任务崩溃
                        ESP_LOGE(TAG, "步进电机处理任务中发生异常");
                        portEXIT_CRITICAL(&stepperMux);
                    }
                }
                else
                {
                    // 超时，检查系统状态
                    ESP_LOGD(TAG, "步进处理任务等待超时，继续等待...");
                }
            }
        }

        // 测试电机功能
        void TestMotor()
        {
            ESP_LOGI(TAG, "开始测试电机功能...");

            // 先测试单个线圈激活
            ESP_LOGI(TAG, "测试线圈1...");
            gpio_set_level(in1_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(in1_pin_, 0);

            ESP_LOGI(TAG, "测试线圈2...");
            gpio_set_level(in2_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(in2_pin_, 0);

            ESP_LOGI(TAG, "测试线圈3...");
            gpio_set_level(in3_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(in3_pin_, 0);

            ESP_LOGI(TAG, "测试线圈4...");
            gpio_set_level(in4_pin_, 1);
            vTaskDelay(pdMS_TO_TICKS(1000));
            gpio_set_level(in4_pin_, 0);

            // 测试简单的步进序列
            ESP_LOGI(TAG, "测试步进序列...");
            Step(1, 256, 10);
            // vTaskDelay(pdMS_TO_TICKS(1000));
            // Step(-1, 1024, 10);
            // 停止电机
            StopMotor();
            ESP_LOGI(TAG, "电机测试完成");
        }

        // 停止电机
        void StopMotor()
        {
            ESP_LOGI(TAG, "停止电机");

            // 停止定时器
            if (gptimer && is_running_)
            {
                esp_err_t err = gptimer_stop(gptimer);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "停止定时器失败: %s", esp_err_to_name(err));
                }
                else
                {
                    ESP_LOGI(TAG, "定时器已停止");
                }
                is_running_ = false;
            }

            // 断电所有线圈
            gpio_set_level(in1_pin_, 0);
            gpio_set_level(in2_pin_, 0);
            gpio_set_level(in3_pin_, 0);
            gpio_set_level(in4_pin_, 0);
            ESP_LOGI(TAG, "已断电所有线圈");
        }

        // 步进电机控制函数
        /*
        direction: 方向，1为顺时针，-1为逆时针
        steps: 步数
        delayTime: 每一步之间的延时，单位为毫秒
        如果电机运转不稳定，可以适当增加 delayTime 参数
        */
        void Step(int direction, int steps, int delayTime)
        {
            ESP_LOGI(TAG, "步进电机控制 - 方向:%d, 步数:%d, 延时:%dms", direction, steps, delayTime);

            // 参数验证
            if (steps <= 0)
            {
                ESP_LOGW(TAG, "步数参数无效: %d", steps);
                return;
            }

            if (delayTime <= 0)
            {
                ESP_LOGW(TAG, "延时参数无效，使用默认值");
                delayTime = DEFAULT_STEP_DELAY_MS;
            }

            int start_position = position_;
            int64_t start_time = esp_timer_get_time() / 1000;

            for (int i = 0; i < steps; i++)
            {
                // 只在访问共享资源时使用临界区
                portENTER_CRITICAL(&stepperMux);

                // 更新步进序列索引
                current_step_ = ((current_step_ + direction) + numSteps) % numSteps;

                // 输出步进序列到GPIO引脚
                gpio_set_level(in1_pin_, stepSequence[current_step_][0]);
                gpio_set_level(in2_pin_, stepSequence[current_step_][1]);
                gpio_set_level(in3_pin_, stepSequence[current_step_][2]);
                gpio_set_level(in4_pin_, stepSequence[current_step_][3]);

                // 更新位置 - 在同一个临界区内更新
                position_ += direction;

                portEXIT_CRITICAL(&stepperMux);

                // 延时 - 在临界区外执行
                vTaskDelay(pdMS_TO_TICKS(delayTime));

                // 每50步打印一次进度
                if (i % 50 == 0 && i > 0)
                {
                    ESP_LOGI(TAG, "步进进度: %d/%d (%.1f%%)", i, steps, (float)i / steps * 100);
                }
            }

            int64_t elapsed_time = (esp_timer_get_time() / 1000) - start_time;
            ESP_LOGI(TAG, "步进完成 - 起始位置:%d, 当前位置:%d, 移动了:%d步, 耗时:%lldms",
                     start_position, position_, position_ - start_position, elapsed_time);
        }

        // 启动电机运行
        void StartMotor(bool open)
        {
            ESP_LOGI(TAG, "收到启动电机请求，状态: %s", is_running_ ? "运行中" : "已停止");

            // 检查定时器是否已初始化
            if (!gptimer)
            {
                ESP_LOGE(TAG, "定时器未初始化，无法启动电机");
                return;
            }

            // 检查步进信号量是否已初始化
            if (!step_semaphore_)
            {
                ESP_LOGE(TAG, "步进信号量未初始化，无法启动电机");
                return;
            }

            // 如果电机正在运行，先停止它
            if (is_running_)
            {
                ESP_LOGW(TAG, "窗户正在移动，将先停止当前操作");
                StopMotor();
                vTaskDelay(pdMS_TO_TICKS(100)); // 等待电机完全停止
            }

            // 设置目标位置
            target_position_ = open ? WINDOW_FULL_STEPS : 0;

            ESP_LOGI(TAG, "启动电机请求 - 当前位置: %d, 目标位置: %d, 当前状态: %s",
                     position_, target_position_, window_opened_ ? "打开" : "关闭");

            // 如果已经在目标状态，无需操作
            if ((open && position_ == WINDOW_FULL_STEPS) ||
                (!open && position_ == 0))
            {
                ESP_LOGI(TAG, "窗户已经在%s状态", open ? "打开" : "关闭");
                return;
            }

            // 设置定时器参数 - 使用单独的字段赋值方式
            gptimer_alarm_config_t alarm_config;
            alarm_config.reload_count = 0;
            alarm_config.alarm_count = (uint64_t)step_delay_ms_ * 1000; // 将毫秒转换为微秒
            alarm_config.flags.auto_reload_on_alarm = true;

            // 检查定时器参数
            if (step_delay_ms_ <= 0)
            {
                ESP_LOGW(TAG, "步进延迟时间无效，使用默认值");
                step_delay_ms_ = DEFAULT_STEP_DELAY_MS;
                alarm_config.alarm_count = step_delay_ms_ * 1000;
            }

            ESP_LOGI(TAG, "配置定时器参数 - 步进延迟: %d ms, 报警计数: %llu",
                     step_delay_ms_, alarm_config.alarm_count);

            // 设置定时器报警动作
            esp_err_t err = gptimer_set_alarm_action(gptimer, &alarm_config);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "设置定时器报警动作失败: %s", esp_err_to_name(err));
                return;
            }

            // 设置运行标志
            is_running_ = true;

            // 启动定时器
            err = gptimer_start(gptimer);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "启动定时器失败: %s", esp_err_to_name(err));
                is_running_ = false;
                return;
            }

            ESP_LOGI(TAG, "开始%s窗户, 步进定时器已启动，步进延迟: %d ms",
                     open ? "打开" : "关闭", step_delay_ms_);

            // 创建监控任务，用于定期检查电机状态
            BaseType_t task_created = xTaskCreate(MotorMonitorTaskFunc, "motor_monitor", 2048, this, 3, nullptr);
            if (task_created != pdPASS)
            {
                ESP_LOGE(TAG, "创建电机监控任务失败");
                // 继续运行，即使监控任务创建失败
            }
            else
            {
                ESP_LOGI(TAG, "电机监控任务创建成功");
            }
        }

        // 电机监控任务函数
        static void MotorMonitorTaskFunc(void *arg)
        {
            WindowController *controller = static_cast<WindowController *>(arg);
            controller->RunMotorMonitorTask();
            vTaskDelete(nullptr);
        }

        // 电机监控任务
        void RunMotorMonitorTask()
        {
            ESP_LOGI(TAG, "电机监控任务开始执行");

            while (is_running_)
            {
                // 每100ms检查一次状态
                vTaskDelay(pdMS_TO_TICKS(100));

                // 更新窗户状态
                UpdateWindowState();

                // 每500ms打印一次日志
                static int log_counter = 0;
                if (++log_counter >= 5)
                {
                    log_counter = 0;
                    ESP_LOGI(TAG, "步进电机移动中 - 位置: %d, 目标: %d", position_, target_position_);
                }

                // 检查是否到达目标位置
                if (position_ == target_position_ || !is_running_)
                {
                    // 停止定时器
                    gptimer_stop(gptimer);

                    // 断电电机
                    StopMotor();

                    ESP_LOGI(TAG, "步进电机任务完成 - 最终位置: %d, 窗户状态: %s",
                             position_, window_opened_ ? "打开" : "关闭");

                    break;
                }
            }
        }

        // 更新窗户状态
        void UpdateWindowState()
        {
            window_opened_ = (position_ >= WINDOW_FULL_STEPS / 2);
        }

    public:
        WindowController(gpio_num_t in1_pin = STEPPER_IN1_PIN,
                         gpio_num_t in2_pin = STEPPER_IN2_PIN,
                         gpio_num_t in3_pin = STEPPER_IN3_PIN,
                         gpio_num_t in4_pin = STEPPER_IN4_PIN)
            : Thing("window_controller", "窗户控制器，支持语音控制"),
              in1_pin_(in1_pin),
              in2_pin_(in2_pin),
              in3_pin_(in3_pin),
              in4_pin_(in4_pin)
        {

            // 初始化GPIO和定时器
            InitializeGpio();

            // 定义窗户状态属性
            properties_.AddBooleanProperty("running", "窗户是否正在移动", [this]() -> bool
                                           { return is_running_; });

            properties_.AddNumberProperty("position", "当前位置(步数)", [this]() -> int
                                          { return position_; });

            properties_.AddBooleanProperty("opened", "窗户是否打开", [this]() -> bool
                                           { return window_opened_; });

            // 定义窗户控制方法（作为语音控制的接口）
            methods_.AddMethod("OpenWindow", "打开窗户", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到打开窗户命令");
                                   StartMotor(true);
                               });

            methods_.AddMethod("CloseWindow", "关闭窗户", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到关闭窗户命令");
                                   StartMotor(false);
                               });

            methods_.AddMethod("ToggleWindow", "切换窗户状态", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到切换窗户状态命令");
                                   StartMotor(!window_opened_);
                                   ESP_LOGI(TAG, "切换窗户状态: %s", window_opened_ ? "关闭" : "打开");
                               });

            methods_.AddMethod("StopWindow", "停止窗户移动", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到停止命令");
                                   // 停止电机
                                   is_running_ = false;
                                   StopMotor();
                               });

            // 添加简化的方法名
            methods_.AddMethod("Open", "打开窗户", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到打开窗户命令");
                                   StartMotor(true);
                               });

            methods_.AddMethod("Close", "关闭窗户", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到关闭窗户命令");
                                   StartMotor(false);
                               });

            methods_.AddMethod("Stop", "停止窗户移动", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "收到停止命令");
                                   is_running_ = false;
                                   StopMotor();
                               });

            // 修改SetSpeed方法，适应定时器控制
            methods_.AddMethod("SetSpeed", "设置窗户移动速度", ParameterList({Parameter("speed", "移动速度(1-10，越大越慢)", kValueTypeNumber, true)}), [this](const ParameterList &params)
                               {
            int speed = params["speed"].number();
            if (speed < 1) speed = 1;
            if (speed > 10) speed = 10;

            // 将速度转换为延迟时间 (1->5ms, 10->50ms)，适合28BYJ-48步进电机
            step_delay_ms_ = speed * 5;
            
            // 如果电机正在运行，更新定时器参数
            if (is_running_ && gptimer) {
                // 停止定时器
                gptimer_stop(gptimer);
                
                // 更新定时器参数
                gptimer_alarm_config_t alarm_config;
                alarm_config.reload_count = 0;
                alarm_config.alarm_count = step_delay_ms_ * 1000;  // 将毫秒转换为微秒
                alarm_config.flags.auto_reload_on_alarm = true;
                
                gptimer_set_alarm_action(gptimer, &alarm_config);
                
                // 重新启动定时器
                gptimer_start(gptimer);
            }
            
            ESP_LOGI(TAG, "窗户移动速度已设置，步进延迟: %d ms", step_delay_ms_); });

            // 添加手动步进控制方法
            methods_.AddMethod("StepForward", "电机正向步进", ParameterList({Parameter("steps", "步数", kValueTypeNumber, false)}), [this](const ParameterList &params)
                               {
            int steps = 1;
            try {
                steps = params["steps"].number();
            } catch (const std::runtime_error&) {
                // 使用默认值
            }

            ESP_LOGI(TAG, "执行正向步进 %d 步", steps);

            // 使用新的Step函数
            Step(1, steps, step_delay_ms_);

            UpdateWindowState(); });

            methods_.AddMethod("StepBackward", "电机反向步进", ParameterList({Parameter("steps", "步数", kValueTypeNumber, false)}), [this](const ParameterList &params)
                               {
            int steps = 1;
            try {
                steps = params["steps"].number();
            } catch (const std::runtime_error&) {
                // 使用默认值
            }

            ESP_LOGI(TAG, "执行反向步进 %d 步", steps);

            // 使用新的Step函数
            Step(-1, steps, step_delay_ms_);

            UpdateWindowState(); });

            // 添加GPIO引脚测试方法
            methods_.AddMethod("TestGPIO", "测试GPIO引脚", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "开始测试GPIO引脚...");
                                   ESP_LOGI(TAG, "使用的GPIO引脚: IN1=%d, IN2=%d, IN3=%d, IN4=%d",
                                            in1_pin_, in2_pin_, in3_pin_, in4_pin_);

                                   // 逐个测试每个引脚
                                   gpio_num_t pins[] = {in1_pin_, in2_pin_, in3_pin_, in4_pin_};
                                   const char *names[] = {"IN1", "IN2", "IN3", "IN4"};

                                   for (int i = 0; i < 4; i++)
                                   {
                                       ESP_LOGI(TAG, "测试引脚 %s (GPIO_%d)", names[i], pins[i]);
                                       gpio_set_level(pins[i], 1);
                                       vTaskDelay(pdMS_TO_TICKS(1000));
                                       gpio_set_level(pins[i], 0);
                                       vTaskDelay(pdMS_TO_TICKS(500));
                                   }
                                   ESP_LOGI(TAG, "GPIO引脚测试完成");
                               });

            // 添加步进序列测试方法
            methods_.AddMethod("TestSequence", "测试步进序列", ParameterList(),
                               [this](const ParameterList &params)
                               {
                                   ESP_LOGI(TAG, "开始测试步进序列...");
                                   for (int i = 0; i < numSteps; i++)
                                   {
                                       portENTER_CRITICAL(&stepperMux);
                                       current_step_ = i;
                                       gpio_set_level(in1_pin_, stepSequence[current_step_][0]);
                                       gpio_set_level(in2_pin_, stepSequence[current_step_][1]);
                                       gpio_set_level(in3_pin_, stepSequence[current_step_][2]);
                                       gpio_set_level(in4_pin_, stepSequence[current_step_][3]);
                                       portEXIT_CRITICAL(&stepperMux);

                                       ESP_LOGI(TAG, "步骤 %d: [%d,%d,%d,%d]",
                                                i,
                                                stepSequence[current_step_][0],
                                                stepSequence[current_step_][1],
                                                stepSequence[current_step_][2],
                                                stepSequence[current_step_][3]);

                                       vTaskDelay(pdMS_TO_TICKS(1000)); // 增加延迟便于观察
                                   }
                                   StopMotor();
                                   ESP_LOGI(TAG, "步进序列测试完成");
                               });

            // 添加连续步进测试方法
            methods_.AddMethod("TestContinuous", "连续步进测试", ParameterList({Parameter("steps", "步数", kValueTypeNumber, false)}), [this](const ParameterList &params)
                               {
            int steps = 20;
            try {
                steps = params["steps"].number();
            } catch (const std::runtime_error&) {
                // 使用默认值
            }

            ESP_LOGI(TAG, "开始连续步进测试，步数: %d", steps);

            // 正向步进
            Step(1, steps, step_delay_ms_);

            // 停止一秒
            StopMotor();
            vTaskDelay(pdMS_TO_TICKS(1000));

            // 反向步进
            Step(-1, steps, step_delay_ms_);

            StopMotor();
            ESP_LOGI(TAG, "连续步进测试完成"); });

            // 添加基本步进测试方法
            methods_.AddMethod("TestStep", "基本步进测试", ParameterList({Parameter("direction", "方向(1正向,-1反向)", kValueTypeNumber, false), Parameter("steps", "步数", kValueTypeNumber, false), Parameter("delay", "延时(ms)", kValueTypeNumber, false)}), [this](const ParameterList &params)
                               {
            int direction = 1;
            int steps = 10;
            int delayTime = step_delay_ms_;

            try {
                direction = params["direction"].number();
                steps = params["steps"].number();
                delayTime = params["delay"].number();
            } catch (const std::runtime_error&) {
                // 使用默认值
            }

            ESP_LOGI(TAG, "基本步进测试 - 方向:%d, 步数:%d, 延时:%dms", direction, steps, delayTime);

            // 使用新的Step函数
            Step(direction, steps, delayTime);

            StopMotor();
            ESP_LOGI(TAG, "基本步进测试完成"); });
        }

        ~WindowController()
        {
            ESP_LOGI(TAG, "销毁窗户控制器");

            // 停止电机
            StopMotor();

            // 删除定时器
            if (gptimer)
            {
                esp_err_t err = gptimer_disable(gptimer);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "禁用定时器失败: %s", esp_err_to_name(err));
                }

                err = gptimer_del_timer(gptimer);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "删除定时器失败: %s", esp_err_to_name(err));
                }
                gptimer = nullptr;
                ESP_LOGI(TAG, "定时器资源已释放");
            }

            // 释放信号量
            if (step_semaphore_)
            {
                vSemaphoreDelete(step_semaphore_);
                step_semaphore_ = nullptr;
                ESP_LOGI(TAG, "步进信号量已释放");
            }

            ESP_LOGI(TAG, "窗户控制器销毁完成");
        }
    };

} // namespace iot

// 注册设备到系统
DECLARE_THING(WindowController);