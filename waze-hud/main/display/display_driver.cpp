#include "display/display_driver.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <algorithm>

namespace waze_hud {
namespace {
constexpr char kTag[] = "DISPLAY";

// Pinout chuẩn cho ESP32-CYD (2.8" ILI9341 SPI)
constexpr gpio_num_t kMosi      = GPIO_NUM_13;
constexpr gpio_num_t kMiso      = GPIO_NUM_12;
constexpr gpio_num_t kSclk      = GPIO_NUM_14;
constexpr gpio_num_t kCs        = GPIO_NUM_15;
constexpr gpio_num_t kDc        = GPIO_NUM_2;
constexpr gpio_num_t kReset     = GPIO_NUM_NC;  // Màn hình CYD nối reset vào nút phần cứng EN (-1)
constexpr gpio_num_t kBacklight = GPIO_NUM_21;

bool onTransferDone(esp_lcd_panel_io_handle_t, esp_lcd_panel_io_event_data_t *, void *ctx) {
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(ctx), &wake);
    return wake == pdTRUE;
}
}  // namespace

DisplayDriver &DisplayDriver::instance() {
    static DisplayDriver driver;
    return driver;
}

esp_err_t DisplayDriver::init() {
    ESP_LOGI(kTag, "Initializing ESP32-CYD ILI9341 SPI panel");

    auto semaphore = xSemaphoreCreateBinary();
    ESP_RETURN_ON_FALSE(semaphore != nullptr, ESP_ERR_NO_MEM, kTag, "Transfer semaphore allocation failed");
    transferDone_ = semaphore;

    // 1. Cấu hình SPI Bus (HSPI / SPI2_HOST trên ESP32)
    spi_bus_config_t buscfg{};
    buscfg.sclk_io_num = kSclk;
    buscfg.mosi_io_num = kMosi;
    buscfg.miso_io_num = kMiso;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    buscfg.max_transfer_sz = layout::MaxRegionPixels * sizeof(uint16_t);
    ESP_RETURN_ON_ERROR(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO), kTag, "SPI bus init failed");

    // 2. Cấu hình Panel IO qua SPI
    esp_lcd_panel_io_spi_config_t ioConfig{};
    ioConfig.cs_gpio_num = kCs;
    ioConfig.dc_gpio_num = kDc;
    ioConfig.spi_mode = 0;
    ioConfig.pclk_hz = 40 * 1000 * 1000; // Tốc độ SPI 40MHz
    ioConfig.trans_queue_depth = 10;
    ioConfig.on_color_trans_done = onTransferDone;
    ioConfig.user_ctx = semaphore;
    ioConfig.lcd_cmd_bits = 8;
    ioConfig.lcd_param_bits = 8;
    ioConfig.flags.swap_color_bytes = 1;

    esp_lcd_panel_io_handle_t io = nullptr;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi(reinterpret_cast<esp_lcd_spi_bus_handle_t>(SPI2_HOST), &ioConfig, &io),
                        kTag, "Panel IO SPI creation failed");
    io_ = io;

    // 3. Khởi tạo driver panel ILI9341
    esp_lcd_panel_dev_config_t panelConfig{};
    panelConfig.reset_gpio_num = kReset;
    panelConfig.rgb_endian = LCD_RGB_ENDIAN_BGR;
    panelConfig.bits_per_pixel = 16;

    esp_lcd_panel_handle_t panel = nullptr;
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_ili9341(io, &panelConfig, &panel), kTag, "ILI9341 driver creation failed");
    panel_ = panel;

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel), kTag, "Panel reset failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel), kTag, "Panel init failed");
    
    // Đảo màu và xoay ngang màn hình (Landscape 320x240)
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(panel, false), kTag, "Invert color failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(panel, true), kTag, "Swap XY failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(panel, false, false), kTag, "Mirror failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel, true), kTag, "Display on failed");

    // 4. Cấu hình điều khiển độ sáng màn hình qua LEDC (PWM)
    ledc_timer_config_t timer{};
    timer.speed_mode = LEDC_LOW_SPEED_MODE;
    timer.timer_num = LEDC_TIMER_0;
    timer.duty_resolution = LEDC_TIMER_10_BIT;
    timer.freq_hz = 5000;
    timer.clk_cfg = LEDC_AUTO_CLK;
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), kTag, "Backlight timer failed");

    ledc_channel_config_t channel{};
    channel.gpio_num = kBacklight;
    channel.speed_mode = LEDC_LOW_SPEED_MODE;
    channel.channel = LEDC_CHANNEL_0;
    channel.intr_type = LEDC_INTR_DISABLE;
    channel.timer_sel = LEDC_TIMER_0;
    channel.duty = 0;
    ESP_RETURN_ON_ERROR(ledc_channel_config(&channel), kTag, "Backlight channel failed");

    ready_ = true;

    // Xóa màn hình khi khởi động
    constexpr int kClearRows = 10;
    auto *clearBuffer = static_cast<uint16_t *>(heap_caps_malloc(
        layout::Width * kClearRows * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    ESP_RETURN_ON_FALSE(clearBuffer != nullptr, ESP_ERR_NO_MEM, kTag, "Clear buffer allocation failed");
    std::fill(clearBuffer, clearBuffer + layout::Width * kClearRows, static_cast<uint16_t>(0x0000));

    for (int y = 0; y < layout::Height; y += kClearRows) {
        const Rect stripe{0, static_cast<int16_t>(y), layout::Width,
                          static_cast<int16_t>(std::min(kClearRows, layout::Height - y))};
        const esp_err_t clearResult = drawRegion(stripe, clearBuffer);
        if (clearResult != ESP_OK) {
            heap_caps_free(clearBuffer);
            ESP_LOGE(kTag, "Startup LCD clear failed at row %d", y);
            return clearResult;
        }
    }
    heap_caps_free(clearBuffer);

    ESP_LOGI(kTag, "ESP32-CYD Display ready");
    return ESP_OK;
}

esp_err_t DisplayDriver::drawRegion(const Rect &region, const uint16_t *pixels) {
    ESP_RETURN_ON_FALSE(ready_ && panel_ != nullptr && pixels != nullptr, ESP_ERR_INVALID_STATE,
                        kTag, "Display is not ready");
    ESP_RETURN_ON_FALSE(region.x >= 0 && region.y >= 0 && region.width > 0 && region.height > 0 &&
                        region.x + region.width <= layout::Width && region.y + region.height <= layout::Height,
                        ESP_ERR_INVALID_ARG, kTag, "Invalid dirty region");

    auto panel = static_cast<esp_lcd_panel_handle_t>(panel_);
    constexpr int kTransferRows = 10;
    for (int row = 0; row < region.height; row += kTransferRows) {
        const int rows = std::min(kTransferRows, region.height - row);
        const uint16_t *stripe = pixels + row * region.width;
        ESP_RETURN_ON_ERROR(esp_lcd_panel_draw_bitmap(panel, region.x, region.y + row,
                                                      region.x + region.width, region.y + row + rows, stripe),
                            kTag, "LCD transfer failed");
        if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(transferDone_), pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(kTag, "LCD transfer timed out at row %d", region.y + row);
            return ESP_ERR_TIMEOUT;
        }
    }
    return ESP_OK;
}

esp_err_t DisplayDriver::setBrightness(uint8_t percent) {
    percent = percent > 100 ? 100 : percent;
    const uint32_t duty = (1023U * percent) / 100U;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty), kTag, "Brightness duty failed");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

esp_err_t DisplayDriver::setOrientation(bool mirrored, bool rotated180) {
    ESP_RETURN_ON_FALSE(ready_ && panel_ != nullptr, ESP_ERR_INVALID_STATE,
                        kTag, "Display is not ready");
    const bool mirrorX = rotated180;
    const bool baseMirrorY = !rotated180;
    const bool mirrorY = mirrored ? !baseMirrorY : baseMirrorY;
    return esp_lcd_panel_mirror(static_cast<esp_lcd_panel_handle_t>(panel_), mirrorX, mirrorY);
}

}  // namespace waze_hud
