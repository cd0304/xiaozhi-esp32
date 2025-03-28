#include "wifi_board.h"
#include "audio_codecs/es8311_audio_codec.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "config.h"
#include "iot/thing_manager.h"

#include <wifi_station.h>
#include <esp_log.h>
#include <esp_efuse_table.h>
#include <driver/i2c_master.h>
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cmath>  // 使用sin函数

// #include <esp_wifi.h>  // 添加这个头文件以使用 esp_wifi_set_ps 函数

// #include <esp_sleep.h>  // 添加此头文件以使用 esp_deep_sleep_start 函数



#include "settings.h"
#include "assets/lang_config.h"
#include "display/lcd_display.h"
#include <esp_lcd_panel_vendor.h>
#include <cstring>  
#define TAG "Esp32c3ChenglongBoard"
LV_FONT_DECLARE(font_puhui_20_4);
LV_FONT_DECLARE(font_awesome_20_4);

// 添加一个标志来跟踪 UART 是否活跃
bool uart_active = false;
// 添加一个标志来跟踪设备是否处于睡眠状态
// bool device_sleeping = false;
// 添加自定义LED类
class ChenglongLed : public Led {
private:
    led_strip_handle_t led_strip_ = nullptr;
    uint8_t r_ = 0, g_ = 0, b_ = 0;
    int blink_counter_ = 0;
    int blink_interval_ms_ = 0;
    esp_timer_handle_t blink_timer_ = nullptr;
    bool enabled_ = true;
    gpio_num_t gpio_pin_;

    esp_timer_handle_t breath_timer_ = nullptr;
    int breath_step_ = 0;
    int breath_max_brightness_ = 40;
    int breath_interval_ms_ = 50;  // 呼吸效果更新间隔
    bool breathing_ = false;
    uint8_t breath_r_ = 0, breath_g_ = 0, breath_b_ = 40;  // 呼吸灯颜色（默认蓝色）


    int breath_count_ = 0;  // 呼吸次数计数
    int max_breath_count_ = 5;  // 最大呼吸次数，达到后进入睡眠

    void StartBlinkTask(int times, int interval_ms) {
        if (led_strip_ == nullptr || !enabled_) {
            return;
        }

        esp_timer_stop(blink_timer_);
        
        blink_counter_ = times * 2;
        blink_interval_ms_ = interval_ms;
        esp_timer_start_periodic(blink_timer_, interval_ms * 1000);
    }

    void OnBlinkTimer() {
        if (!enabled_ || led_strip_ == nullptr) {
            return;
        }
        
        blink_counter_--;
        if (blink_counter_ & 1) {
            led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
            led_strip_refresh(led_strip_);
        } else {
            led_strip_clear(led_strip_);

            if (blink_counter_ == 0) {
                esp_timer_stop(blink_timer_);
            }
        }
    }
    // 呼吸灯效果处理函数
    void OnBreathTimer() {
        if (led_strip_ == nullptr || !enabled_) {
            return;
        }
        
        // 使用正弦函数创造平滑的呼吸效果
        // 将breath_step_映射到0-2π区间，使用正弦函数产生完整的波形
        // sin(x)的值域是[-1,1]，我们需要将其映射到[0,1]
        float angle = breath_step_ * 3.14159f / 100.0f;  // 0到2π的映射
        float brightness_factor = (sin(angle) + 1.0f) / 2.0f;  // 将[-1,1]映射到[0,1]
        
        // 打印调试信息，查看亮度变化
        // ESP_LOGI(TAG, "呼吸灯步进: %d, 亮度因子: %.2f", breath_step_, brightness_factor);
        
        // 更新LED亮度
        uint8_t r = breath_r_ * brightness_factor;
        uint8_t g = breath_g_ * brightness_factor;
        uint8_t b = breath_b_ * brightness_factor;
        
        led_strip_set_pixel(led_strip_, 0, r, g, b);
        led_strip_refresh(led_strip_);
        
        // 更新步进值
        breath_step_++;
        if (breath_step_ >= 200) {  // 完成一个完整的呼吸周期
            breath_step_ = 0;
            breath_count_++;  // 增加呼吸次数计数
            
            ESP_LOGI(TAG, "呼吸周期完成，当前次数: %d/%d", breath_count_, max_breath_count_);
            
            // 检查是否达到最大呼吸次数
            if (max_breath_count_ > 0 && breath_count_ >= max_breath_count_) {
                // ESP_LOGI(TAG, "达到最大呼吸次数 %d，准备进入深度睡眠", max_breath_count_);
                // breath_count_ = 0 ;
                // // 停止呼吸定时器
                // esp_timer_stop(breath_timer_);
                
                // // 创建一个延迟任务来执行睡眠，以便让日志完成输出
                // esp_timer_handle_t sleep_timer;
                // esp_timer_create_args_t timer_args = {
                //     .callback = [](void *arg) {
                //         ESP_LOGI(TAG, "进入轻度睡眠模式...");
                        
                //         // 关闭LED
                //         auto led = static_cast<ChenglongLed*>(arg);
                //         led->TurnOff();
                        
                //         // 配置唤醒源（可以通过GPIO唤醒）
                //         // esp_sleep_enable_ext0_wakeup(GPIO_NUM_9, 0); // BOOT按钮低电平唤醒
                        
                //         // 进入深度睡眠
                //         // esp_deep_sleep_start();

                //         // 设置全局睡眠标志
                //         device_sleeping = true;
                //         esp_light_sleep_start(); // 进入轻度睡眠

                //         // 唤醒后的处理
                //         ESP_LOGI(TAG, "设备从轻度睡眠中唤醒");
                //         device_sleeping = false;
                //         // breath_count_ = 0 ;
                //         // 重新启用LED
                //         // led->Enable();
                //         // led->OnStateChanged();
                //     },
                //     .arg = this,
                //     .dispatch_method = ESP_TIMER_TASK,
                //     .name = "sleep_timer",
                //     .skip_unhandled_events = false,
                // };
                // esp_timer_create(&timer_args, &sleep_timer);
                // esp_timer_start_once(sleep_timer, 1000 * 1000); // 1秒后进入睡眠
            }
           
        }
    }

    void SetColor(uint8_t r, uint8_t g, uint8_t b) {
        r_ = r;
        g_ = g;
        b_ = b;
    }

public:
    ChenglongLed(gpio_num_t gpio) : gpio_pin_(gpio) {
        // 初始化LED控制器
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = gpio;
        strip_config.max_leds = 1;
        strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "无法创建LED控制器: %s", esp_err_to_name(err));
            led_strip_ = nullptr;
            enabled_ = false;
            return;
        }
        
        led_strip_clear(led_strip_);

        // 创建闪烁定时器
        esp_timer_create_args_t blink_timer_args = {
            .callback = [](void *arg) {
                auto led = static_cast<ChenglongLed*>(arg);
                led->OnBlinkTimer();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "blink_timer",
            .skip_unhandled_events = false,
        };
        
        err = esp_timer_create(&blink_timer_args, &blink_timer_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "无法创建闪烁定时器: %s", esp_err_to_name(err));
            if (led_strip_ != nullptr) {
                led_strip_del(led_strip_);
                led_strip_ = nullptr;
            }
            enabled_ = false;
            return;
        }

        // 创建呼吸灯定时器
        ESP_LOGI(TAG, "在构造函数中创建呼吸定时器");
        esp_timer_create_args_t breath_timer_args = {
            .callback = [](void *arg) {
                // ESP_LOGI(TAG, "呼吸定时器回调触发");
                auto led = static_cast<ChenglongLed*>(arg);
                led->OnBreathTimer();
            },
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "breath_timer",
            .skip_unhandled_events = false,
        };
        
        err = esp_timer_create(&breath_timer_args, &breath_timer_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "无法创建呼吸灯定时器: %s", esp_err_to_name(err));
            breath_timer_ = nullptr;
        } else {
            ESP_LOGI(TAG, "呼吸定时器创建成功: %p", breath_timer_);
        }
    }
    // 启动呼吸灯效果
    void StartBreathing(uint8_t r = 0, uint8_t g = 0, uint8_t b = 40) {
        if (led_strip_ == nullptr || !enabled_) {
            ESP_LOGW(TAG, "LED不可用，无法启动呼吸效果");
            return;
        }
        
        // 停止闪烁定时器
        if (blink_timer_ != nullptr) {
            esp_timer_stop(blink_timer_);
        }
        
        // 停止之前的呼吸定时器
        if (breath_timer_ != nullptr) {
            esp_timer_stop(breath_timer_);
        } else {
            // 如果定时器未创建，创建新定时器
            ESP_LOGI(TAG, "创建新的呼吸定时器");
            esp_timer_create_args_t breath_timer_args = {
                .callback = [](void *arg) {
                    auto led = static_cast<ChenglongLed*>(arg);
                    led->OnBreathTimer();
                },
                .arg = this,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "breath_timer",
                .skip_unhandled_events = false,
            };
            
            esp_err_t err = esp_timer_create(&breath_timer_args, &breath_timer_);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "无法创建呼吸定时器: %s", esp_err_to_name(err));
                return;
            }
        }
        
        // 设置呼吸灯颜色
        breath_r_ = r;
        breath_g_ = g;
        breath_b_ = b;
        
        // 重置呼吸步进
        breath_step_ = 0;
        breathing_ = true;
        

        // 重置呼吸步进和计数
        breath_step_ = 0;
        breath_count_ = 0;
        max_breath_count_ = max_breath_count_;
        breathing_ = true;

        // 启动呼吸灯定时器
        ESP_LOGI(TAG, "启动呼吸灯效果，颜色: (%d,%d,%d), 间隔: %dms", 
                 breath_r_, breath_g_, breath_b_, breath_interval_ms_);
        
        // 添加测试代码，验证回调是否工作
        ESP_LOGI(TAG, "定时器句柄: %p", breath_timer_);
        
        // 确保定时器正确启动
        esp_err_t start_err = esp_timer_start_periodic(breath_timer_, breath_interval_ms_ * 1000);
        if (start_err != ESP_OK) {
            ESP_LOGE(TAG, "启动呼吸定时器失败: %s", esp_err_to_name(start_err));
        } else {
            ESP_LOGI(TAG, "呼吸定时器启动成功");
            
            // 添加测试回调，确认定时器工作
            vTaskDelay(pdMS_TO_TICKS(100));
            ESP_LOGI(TAG, "100ms后，呼吸步进应该已经更新: %d", breath_step_);
        }
    }
    
    // 停止呼吸灯效果
    void StopBreathing() {
        if (breath_timer_ == nullptr) {
            return;
        }
        
        breathing_ = false;
        esp_timer_stop(breath_timer_);
        led_strip_clear(led_strip_);
        ESP_LOGI(TAG, "呼吸灯效果已停止");
    }
    ~ChenglongLed() {
        if (blink_timer_ != nullptr) {
            esp_timer_stop(blink_timer_);
        }
        if (led_strip_ != nullptr) {
            led_strip_del(led_strip_);
        }

        if (breath_timer_ != nullptr) {
            esp_timer_stop(breath_timer_);
            esp_timer_delete(breath_timer_);
        }
    }

    void OnStateChanged() override {
        if (!enabled_) {
            return;
        }
        
        auto& app = Application::GetInstance();
        auto device_state = app.GetDeviceState();

            // 使用静态变量记录启动时间
        static uint32_t startup_time = esp_timer_get_time() / 1000000; // 秒
    
        switch (device_state) {
            case kDeviceStateStarting:
                SetColor(0, 0, 40);
                StartContinuousBlink(100);
                break;
            case kDeviceStateWifiConfiguring:
                SetColor(0, 0, 40);
                StartContinuousBlink(500);
                break;
            case kDeviceStateIdle:
            {   // 注意这里添加了花括号创建局部作用域
                ESP_LOGI(TAG, "设备状态为Idle，设置颜色为(0, 0, 40)");
                SetColor(0, 0, 40);
                
                // 检查是否已经过了启动后5秒
                uint32_t current_time = esp_timer_get_time() / 1000000; // 秒
                if (current_time - startup_time >= 5) {
                    // 已经过了启动后5秒，可以安全启动呼吸灯
                    StartBreathing();
                } else {
                    // 尚未过启动后5秒，先关闭LED
                    ESP_LOGI(TAG, "系统启动后 %lu 秒，暂时关闭LED并等待", current_time - startup_time);
                    TurnOff();
                    
                    // 计算还需等待的时间（至少2秒，最多7秒）
                    // 修复：使用模板参数指定类型
                    uint32_t wait_time = std::max<uint32_t>(2000U, (7 - (current_time - startup_time)) * 1000);
                    
                    // 创建一次性定时器
                    static esp_timer_handle_t delayed_breath_timer = nullptr;
                    
                    // 如果已存在定时器，先停止并删除
                    if (delayed_breath_timer != nullptr) {
                        esp_timer_stop(delayed_breath_timer);
                        esp_timer_delete(delayed_breath_timer);
                        delayed_breath_timer = nullptr;
                    }
                    
                    // 创建新定时器
                    esp_timer_create_args_t timer_args = {
                        .callback = [](void *arg) {
                            auto led = static_cast<ChenglongLed*>(arg);
                            ESP_LOGI(TAG, "延迟后启动呼吸灯");
                            led->StartBreathing();
                            delayed_breath_timer = nullptr;  // 清除静态引用
                        },
                        .arg = this,
                        .dispatch_method = ESP_TIMER_TASK,
                        .name = "delayed_breath",
                        .skip_unhandled_events = false,
                    };
                    esp_timer_create(&timer_args, &delayed_breath_timer);
                    esp_timer_start_once(delayed_breath_timer, wait_time * 1000);

                }
                break;
            }

            case kDeviceStateConnecting:
                SetColor(0, 0, 40);
                TurnOn();
                break;
            case kDeviceStateListening:
                if (app.IsVoiceDetected()) {
                    SetColor(100, 0, 0);
                } else {
                    SetColor(20, 0, 0);
                }
                TurnOn();
                break;
            case kDeviceStateSpeaking:
                SetColor(0, 40, 0);
                TurnOn();
                break;
            case kDeviceStateUpgrading:
                SetColor(0, 40, 0);
                StartContinuousBlink(100);
                break;
            case kDeviceStateActivating:
                SetColor(0, 40, 0);
                StartContinuousBlink(500);
                break;
            default:
                ESP_LOGW(TAG, "未知LED事件: %d", device_state);
                return;
        }
    }

    void TurnOn() {
       
        if (led_strip_ == nullptr || !enabled_) {
            return;
        }
        if (breathing_) {
            StopBreathing();
        }
        led_strip_set_pixel(led_strip_, 0, r_, g_, b_);
        led_strip_refresh(led_strip_);
    }

    void TurnOff() {
        if (led_strip_ == nullptr || !enabled_) {
            return;
        }
        SetColor(0, 0, 0);
        esp_timer_stop(blink_timer_);
        led_strip_clear(led_strip_);

    }

    void BlinkOnce() {
        Blink(1, 100);
    }

    void Blink(int times, int interval_ms) {
        StartBlinkTask(times, interval_ms);
    }

    void StartContinuousBlink(int interval_ms) {
        StartBlinkTask(-1, interval_ms);
    }

    void Disable() {
        ESP_LOGI(TAG, "禁用LED控制器");
        
        if (led_strip_ == nullptr) {
            return;
        }
        
        // 停止所有定时器
        if (blink_timer_ != nullptr) {
            esp_timer_stop(blink_timer_);
        }
        if (breath_timer_ != nullptr) {
            esp_timer_stop(breath_timer_);
        }
        
        // 清除LED显示
        led_strip_clear(led_strip_);
        
        // 删除LED控制器，释放GPIO
        led_strip_del(led_strip_);
        led_strip_ = nullptr;
        
        enabled_ = false;
        ESP_LOGI(TAG, "LED控制器已禁用");
    }

    void Enable() {
        if (enabled_ && led_strip_ != nullptr) {
            ESP_LOGI(TAG, "LED已启用，无需重复启用");
            return;
        }
        
        // 保存当前状态
        bool was_breathing = breathing_;
        uint8_t saved_breath_r = breath_r_;
        uint8_t saved_breath_g = breath_g_;
        uint8_t saved_breath_b = breath_b_;
        
        // 确保GPIO引脚处于正确状态
        gpio_reset_pin(gpio_pin_);
        gpio_set_direction(gpio_pin_, GPIO_MODE_OUTPUT);
        vTaskDelay(pdMS_TO_TICKS(10));
        
        led_strip_config_t strip_config = {};
        strip_config.strip_gpio_num = gpio_pin_;
        strip_config.max_leds = 1;
        strip_config.led_pixel_format = LED_PIXEL_FORMAT_GRB;
        strip_config.led_model = LED_MODEL_WS2812;

        led_strip_rmt_config_t rmt_config = {};
        rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz

        esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip_);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "无法创建LED控制器: %s", esp_err_to_name(err));
            led_strip_ = nullptr;
            return;
        }
        
        enabled_ = true;
        ESP_LOGI(TAG, "LED控制器已成功启用");
        
        // 恢复之前的状态
        if (was_breathing) {
            ESP_LOGI(TAG, "恢复之前的呼吸效果");
            StartBreathing(saved_breath_r, saved_breath_g, saved_breath_b);
        } else {
            // 恢复当前设备状态对应的LED状态
            OnStateChanged();
        }
    }

    bool IsEnabled() const {
        return enabled_ && led_strip_ != nullptr;
    }

    // 添加公共访问方法
    bool IsBreathing() const { return breathing_; }
    uint8_t GetBreathR() const { return breath_r_; }
    uint8_t GetBreathG() const { return breath_g_; }
    uint8_t GetBreathB() const { return breath_b_; }
    
    // 添加保存和恢复状态的方法
    struct LedState {
        bool was_breathing;
        uint8_t breath_r, breath_g, breath_b;
    };
    
    LedState GetState() const {
        return {
            .was_breathing = breathing_,
            .breath_r = breath_r_,
            .breath_g = breath_g_,
            .breath_b = breath_b_
        };
    }
    
    void RestoreState(const LedState& state) {
        if (state.was_breathing) {
            StartBreathing(state.breath_r, state.breath_g, state.breath_b);
        } else {
            OnStateChanged();
        }
    }
};

class Esp32c3ChenglongBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t codec_i2c_bus_;
    Button boot_button_;
    TaskHandle_t uart_task_handle_;
    bool press_to_talk_enabled_ = false;

    LcdDisplay* display_;
    ChenglongLed* led_strip_ = nullptr;
    
    void InitializeCodecI2c() {
        // Initialize I2C peripheral
        i2c_master_bus_config_t i2c_bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = AUDIO_CODEC_I2C_SDA_PIN,
            .scl_io_num = AUDIO_CODEC_I2C_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &codec_i2c_bus_));
    }

    static void UartListenTask(void* arg) {
        Esp32c3ChenglongBoard* board = static_cast<Esp32c3ChenglongBoard*>(arg);
        uint8_t data[64]; // 增大缓冲区
        uint8_t packet_buffer[16]; // 用于存储单个完整数据包
        int buffer_index = 0;
        bool packet_start = false;
        
        ESP_LOGI(TAG, "UART listen task started");

        while (true) {
            memset(data, 0, sizeof(data));
            int length = uart_read_bytes(UART_NUM_0, data, sizeof(data), pdMS_TO_TICKS(100));
            
            if (length > 0) {
                // 逐字节处理接收到的数据
                for (int i = 0; i < length; i++) {
                    // 检测数据包开始标志 (0xA5 0xFA)
                    if (data[i] == 0xA5 && i + 1 < length && data[i + 1] == 0xFA) {
                        // 发现新的数据包开始，重置缓冲区
                        buffer_index = 0;
                        packet_start = true;
                        packet_buffer[buffer_index++] = data[i]; // 存储 A5
                        packet_buffer[buffer_index++] = data[i + 1]; // 存储 FA
                        i++; // 跳过已处理的 FA
                        continue;
                    }
                    
                    // 如果已经找到数据包开始标志，继续存储数据
                    if (packet_start && buffer_index < sizeof(packet_buffer)) {
                        packet_buffer[buffer_index++] = data[i];
                        
                        // 检查是否到达数据包结束标志 (FB)
                        if ((data[i] == 0xFB ) && buffer_index >= 8) {
                            // 处理完整的数据包
                            board->ProcessPacket(packet_buffer, buffer_index);
                            // 重置状态，准备接收下一个数据包
                            packet_start = false;
                            buffer_index = 0;
                        }
                    }
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // void wakeup_from_uart() {
    //     // 如果设备处于睡眠状态，先唤醒设备

    //         ESP_LOGI(TAG, "设备正在从睡眠状态唤醒");
    //         // 设备会通过 UART 中断自动唤醒，这里只需重置状态
    //         device_sleeping = false;
            
    //         // 重新启用LED
    //         if (led_strip_) {
    //             led_strip_->Enable();
    //             led_strip_->OnStateChanged();
    //         }
            
    //         // 短暂延迟，确保设备完全唤醒
    //         vTaskDelay(pdMS_TO_TICKS(100));
        
        
    // }
    // 新增方法：处理完整的数据包
    void ProcessPacket(const uint8_t* packet, int length) {
        // 检查数据包长度是否合法
        if (length < 8) {
            ESP_LOGW(TAG, "数据包长度不足: %d bytes", length);
            return;
        }
        
        // 验证数据包头部
        if (packet[0] != 0xA5 || packet[1] != 0xFA) {
            ESP_LOGW(TAG, "数据包头部无效");
            return;
        }
        
        // 打印接收到的数据包信息
        ESP_LOGI(TAG, "接收到数据包 (%d bytes):", length);
        
        // 握手请求 A5 FA 00 82 01 00 20 FB
        if (length == 8 && packet[2] == 0x00 && packet[3] == 0x82 && 
            packet[4] == 0x01 && packet[5] == 0x00 && 
            packet[6] == 0x20 && packet[7] == 0xFB) {
            
            // 发送握手响应
            uint8_t response[] = {0xA5, 0xFA, 0x00, 0x82, 0x01, 0x00, 0x21, 0xFB};
            SendUart(response, sizeof(response));
            ESP_LOGI(TAG, "发送握手响应");
        }
        // 唤醒请求 A5 FA 00 81 01 00 21 FB
        else if (length == 8 && packet[2] == 0x00 && packet[3] == 0x81 && 
                 packet[4] == 0x01 && packet[5] == 0x00 && 
                 packet[6] == 0x21 && packet[7] == 0xFB) {
            
            // 发送唤醒响应 A5 FA 00 82 01 00 22 FB
            // uint8_t response[] = {0xA5, 0xFA, 0x00, 0x82, 0x01, 0x00, 0x22, 0xFB};
            // SendUart(response, sizeof(response));
            ESP_LOGI(TAG, "收到唤醒词，开始对话");
            // 如果设备处于睡眠状态，先唤醒设备
            // if (device_sleeping) {
            //    wakeup_from_uart();
            // }
        
            
            Application::GetInstance().WakeWordInvoke("你好");
        }
        // 按键请求 A5 FA 00 82 01 00 0A FB
        else if (length == 8 && packet[2] == 0x00 && packet[3] == 0x82 && 
                 packet[4] == 0x01 && packet[5] == 0x00 && 
                 packet[6] == 0x0A && packet[7] == 0xFB) {
            ESP_LOGI(TAG, "收到按键请求，开始对话");
            // if (device_sleeping) {
            //    wakeup_from_uart();
            // }
            Application::GetInstance().WakeWordInvoke("你好");
        }
         // 关机请求 A5 FA 00 82 01 00 0F FB
        else if (length == 8 && packet[2] == 0x00 && packet[3] == 0x82 && 
                 packet[4] == 0x01 && packet[5] == 0x00 && 
                 packet[6] == 0x0F && packet[7] == 0xFB) {
            ESP_LOGI(TAG, "收到关机请求，播放关机声音，向服务器发bye");
            Application::GetInstance().PlaySound(Lang::Sounds::P3_EXCLAMATION);

        }
        else {
            // 未知数据包
            ESP_LOGW(TAG, "未知数据包类型");
            // 可以添加调试信息，打印数据包内容
            for (int i = 0; i < length; i++) {
                ESP_LOGD(TAG, "byte[%d]: 0x%02X", i, packet[i]);
            }
        }
    }
     // 添加串口初始化函数
    void InitializeUart() {
        esp_log_level_set("uart", ESP_LOG_NONE);
        uart_config_t uart_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };
    
        ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 256, 0, NULL, 0));
        ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
        ESP_ERROR_CHECK(uart_set_pin(UART_NUM_0, GPIO_NUM_21, GPIO_NUM_20, 
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

        // 允许 UART 唤醒
        // esp_sleep_enable_uart_wakeup(UART_NUM_0);
        
        // 配置 UART 为唤醒源
        // ESP_ERROR_CHECK(uart_set_wakeup_threshold(UART_NUM_0, 3)); // 接收到3个字节时唤醒

        
        // ESP_ERROR_CHECK(uart_set_rx_timeout(UART_NUM_0, 10)); // 设置接收超时
        // ESP_ERROR_CHECK(uart_set_rx_full_threshold(UART_NUM_0, 8)); // 设置接收缓冲区阈值
        
        // ESP_LOGI(TAG, "UART initialized successfully with wakeup capability");

        //  强制使用 XTAL 作为时钟源
        // esp_pm_config_esp32c3_t pm_config = {
        //     .max_freq_mhz = 80,   // 最高频率
        //     .min_freq_mhz = 10,   // 最低频率
        //     .light_sleep_enable = true  // 启用 Light Sleep
        // };
        // esp_pm_configure(&pm_config);
        // // 关闭动态电源管理
        // esp_pm_lock_handle_t pm_lock;
        // esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "disable_dpm", &pm_lock);
        // esp_pm_lock_acquire(pm_lock);  // 锁定最高频率，避免动态降频

        // ESP_LOGI(TAG, "UART initialized successfully");
        // 创建串口监听任务，增加栈大小
        xTaskCreate(UartListenTask,          // 任务函数
                   "uart_task",              // 任务名称
                   2048,                     // 任务堆栈大小，从2048增加到4096
                   this,                     // 传递给任务的参数
                   5,                        // 任务优先级
                   &uart_task_handle_);      // 任务句柄
    }

    void SendUart(const uint8_t* response, size_t length) {
        ESP_LOGI(TAG, "准备发送UART响应");
        
        // 保存LED状态 - 使用静态变量避免堆分配
        static ChenglongLed::LedState led_state;
        static bool has_led_state = false;
        
        if (led_strip_) {
            led_state = led_strip_->GetState();
            has_led_state = true;
            
            // 禁用LED
            led_strip_->Disable();
             // 等待一小段时间确保彩灯信号完全停止
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            has_led_state = false;
        }
        
        // 标记UART为活跃状态
        uart_active = true;
        
        // 配置UART引脚
        uart_set_pin(UART_NUM_0, GPIO_NUM_21, GPIO_NUM_20, 
                     UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        vTaskDelay(pdMS_TO_TICKS(10));
        uart_flush(UART_NUM_0);
        // 发送数据
        uint8_t buffer[length + 1];
        buffer[0] = length;
        memcpy(buffer + 1, response, length);
        
        uart_write_bytes(UART_NUM_0, buffer, length + 1);
        uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
        
        // 标记UART为非活跃状态
        uart_active = false;
        
        // 使用静态任务函数
        static ChenglongLed* led_to_restore = led_strip_;
        
        // 创建任务使用普通的C函数
        xTaskCreate(
            // 静态任务函数
            [](void* arg) {
                vTaskDelay(pdMS_TO_TICKS(500));
                
                if (!uart_active && led_to_restore && has_led_state) {
                    ESP_LOGI(TAG, "重新启用LED控制器");
                    led_to_restore->Enable();
                    
                    if (led_to_restore->IsEnabled()) {
                        ESP_LOGI(TAG, "恢复LED状态");
                        led_to_restore->RestoreState(led_state);
                    }
                }
                
                vTaskDelete(NULL);
            },
            "led_enable_task", 
            4096,           // 栈大小
            NULL,           // 参数
            1,              // 优先级
            NULL            // 任务句柄
        );
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()) {
                ResetWifiConfiguration();
            }
            if (!press_to_talk_enabled_) {
                app.ToggleChatState();
            }
        });
        boot_button_.OnPressDown([this]() {
            if (press_to_talk_enabled_) {
                Application::GetInstance().StartListening();
            }
        });
        boot_button_.OnPressUp([this]() {
            if (press_to_talk_enabled_) {
                Application::GetInstance().StopListening();
            }
        });
    }

    // 物联网初始化，添加对 AI 可见设备
    void InitializeIot() {
        Settings settings("vendor");
        press_to_talk_enabled_ = settings.GetInt("press_to_talk", 0) != 0;

        auto& thing_manager = iot::ThingManager::GetInstance();
        thing_manager.AddThing(iot::CreateThing("Speaker"));
        thing_manager.AddThing(iot::CreateThing("MyThing"));


    }

    void InitializeSt7789Display() {
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        // 液晶屏控制IO初始化
        ESP_LOGD(TAG, "Install panel IO");
        esp_lcd_panel_io_spi_config_t io_config = {};
        io_config.cs_gpio_num = DISPLAY_SPI_CS_PIN;
        io_config.dc_gpio_num = DISPLAY_DC_PIN;
        io_config.spi_mode = 2;
        io_config.pclk_hz = 80 * 1000 * 1000;
        io_config.trans_queue_depth = 10;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &panel_io));

        // 初始化液晶屏驱动芯片ST7789
        ESP_LOGD(TAG, "Install LCD driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));
        
        esp_lcd_panel_reset(panel);

        esp_lcd_panel_init(panel);
        esp_lcd_panel_invert_color(panel, true);
        esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY);
        esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
        display_ = new SpiLcdDisplay(panel_io, panel,
                                DISPLAY_WIDTH, DISPLAY_HEIGHT, 
                                DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y, 
                                DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, 
                                DISPLAY_SWAP_XY,
                                {
                                    .text_font = &font_puhui_20_4,
                                    .icon_font = &font_awesome_20_4,
                                    .emoji_font = font_emoji_32_init(),
                                });
    }

public:
    Esp32c3ChenglongBoard() : boot_button_(BOOT_BUTTON_GPIO) {  
        // 把 ESP32C3 的 VDD SPI 引脚作为普通 GPIO 口使用
        esp_efuse_write_field_bit(ESP_EFUSE_VDD_SPI_AS_GPIO);

        InitializeCodecI2c();
        InitializeButtons();

        auto codec = GetAudioCodec();
        // 添加延迟，让系统有时间初始化音频组件
        vTaskDelay(pdMS_TO_TICKS(100));

        InitializeIot();
        InitializeUart();  // 添加串口初始化

        InitializeSpi();//tft显示屏
        InitializeSt7789Display();

       
        // codec->SetOutputVolume(90);
        // GetBacklight()->SetBrightness(70);

        // esp_wifi_set_max_tx_power(12); //当设备与路由器距离较近（<5米）时，可以降低功率节省电量。（默认 20dBm）。
        // esp_wifi_set_ps(WIFI_PS_MIN_MODEM); //ESP32-C3 提供 Modem-sleep 模式，在 Wi-Fi 空闲时降低功耗.适用场景：Wi-Fi 并非持续高流量传输时，如在语音数据交换的间隔期间省电。
    }

    void InitializeSpi() {
        spi_bus_config_t buscfg = {};
        buscfg.mosi_io_num = DISPLAY_SPI_MOSI_PIN;
        buscfg.miso_io_num = GPIO_NUM_NC;
        buscfg.sclk_io_num = DISPLAY_SPI_SCK_PIN;
        buscfg.quadwp_io_num = GPIO_NUM_NC;
        buscfg.quadhd_io_num = GPIO_NUM_NC;
        buscfg.max_transfer_sz = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    }

    virtual Led* GetLed() override {
        ESP_LOGI(TAG, "GetLed");
        if (!led_strip_) {
            led_strip_ = new ChenglongLed(BUILTIN_LED_GPIO);
        }
        return led_strip_;
    }
    virtual Display* GetDisplay() override {
        return display_;
    }

    // virtual AudioCodec* GetAudioCodec() override {
    //     // static Es8311AudioCodec audio_codec(codec_i2c_bus_, I2C_NUM_0, AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
    //     //     AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
    //     //     AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
    //     // return &audio_codec;

    //     static Es8311AudioCodec* audio_codec = nullptr;
    
    //     if (audio_codec == nullptr) {
    //         // 尝试初始化音频编解码器
    //         audio_codec = new Es8311AudioCodec(codec_i2c_bus_, I2C_NUM_0, 
    //             AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
    //             AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, 
    //             AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
    //             AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
                
    //         ESP_LOGI(TAG, "音频编解码器初始化完成");
    //     }
        
    //     return audio_codec;
    // }
    virtual AudioCodec* GetAudioCodec() override {
        static Es8311AudioCodec* audio_codec = nullptr;
        static int init_attempts = 0;
        const int max_attempts = 3;
        
        if (audio_codec == nullptr && init_attempts < max_attempts) {
            init_attempts++;
            ESP_LOGI(TAG, "尝试初始化音频编解码器 (尝试 %d/%d)", init_attempts, max_attempts);
            
            try {
                // 添加更多内存检查
                size_t free_heap = esp_get_free_heap_size();
                ESP_LOGI(TAG, "当前可用堆内存: %d 字节", free_heap);
                
                if (free_heap < 10000) {  // 假设需要至少10KB内存
                    ESP_LOGW(TAG, "可用内存不足，可能导致初始化失败");
                }
                
                // 尝试初始化音频编解码器
                audio_codec = new Es8311AudioCodec(codec_i2c_bus_, I2C_NUM_0, 
                    AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
                    AUDIO_I2S_GPIO_MCLK, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, 
                    AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN,
                    AUDIO_CODEC_PA_PIN, AUDIO_CODEC_ES8311_ADDR);
                    
                ESP_LOGI(TAG, "音频编解码器初始化完成");
            } catch (const std::exception& e) {
                ESP_LOGE(TAG, "音频编解码器初始化异常: %s", e.what());
                delete audio_codec;
                audio_codec = nullptr;
            } catch (...) {
                ESP_LOGE(TAG, "音频编解码器初始化发生未知异常");
                delete audio_codec;
                audio_codec = nullptr;
            }
        }
        
        if (audio_codec == nullptr && init_attempts >= max_attempts) {
            ESP_LOGE(TAG, "音频编解码器初始化失败，已达到最大尝试次数");
            
            // 创建一个空的音频编解码器，避免空指针异常
            // static DummyAudioCodec dummy_codec;
            // return &dummy_codec;
        }
        
        return audio_codec;
    }

    void SetPressToTalkEnabled(bool enabled) {
        press_to_talk_enabled_ = enabled;

        Settings settings("vendor", true);
        settings.SetInt("press_to_talk", enabled ? 1 : 0);
        ESP_LOGI(TAG, "Press to talk enabled: %d", enabled);
    }

    bool IsPressToTalkEnabled() {
        return press_to_talk_enabled_;
    }
    virtual Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }



    //iot 指令，让小智关机
    void TurnOffDevice() {
        ESP_LOGI(TAG, "收到关机命令:");
        //循环检测3秒，小智的状态是否speaking状态
        int64_t start_time = esp_timer_get_time();
        while (Application::GetInstance().GetDeviceState() == DeviceState::kDeviceStateSpeaking) {
            ESP_LOGI(TAG, "小智正在说话，等待其结束");
            //让led闪烁一次
            led_strip_->BlinkOnce();
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            if (esp_timer_get_time() - start_time > 3000000) { // 超过2秒
                break;
            }
        }


        Application::GetInstance().PlaySound(Lang::Sounds::P3_EXCLAMATION);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 等一下叮咚的关机声音再关机

        uint8_t response[] = {0xA5, 0xFA, 0x00, 0x82, 0x01, 0x00, 0xFF, 0xFB};
        SendUart(response, sizeof(response));

    }
    // 析构函数中释放资源
    ~Esp32c3ChenglongBoard() {
        if (led_strip_) {
            delete led_strip_;
        }
    }
};

DECLARE_BOARD(Esp32c3ChenglongBoard);



namespace iot {

class MyThing : public Thing {
public:
    MyThing() : Thing("MyThing", "控制对话模式，一种是长按对话，一种是单击后连续对话,同时还能控制远程关机") {
        // 定义设备的属性
        properties_.AddBooleanProperty("enabled", "true 表示长按说话模式，false 表示单击说话模式", []() -> bool {
            auto board = static_cast<Esp32c3ChenglongBoard*>(&Board::GetInstance());
            return board->IsPressToTalkEnabled();
        });

        // 定义设备可以被远程执行的指令
        methods_.AddMethod("SetEnabled", "启用或禁用长按说话模式，调用前需要经过用户确认", ParameterList({
            Parameter("enabled", "true 表示长按说话模式，false 表示单击说话模式", kValueTypeBoolean, true)
        }), [](const ParameterList& parameters) {
            bool enabled = parameters["enabled"].boolean();
            auto board = static_cast<Esp32c3ChenglongBoard*>(&Board::GetInstance());
            board->SetPressToTalkEnabled(enabled);
        });

        methods_.AddMethod("TurnOff", "关机", ParameterList(), [this](const ParameterList& parameters) {
            auto board = static_cast<Esp32c3ChenglongBoard*>(&Board::GetInstance());
            board->TurnOffDevice();
        });
    }
};

} // namespace iot

DECLARE_THING(MyThing);
