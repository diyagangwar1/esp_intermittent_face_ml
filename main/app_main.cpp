#include "esp_log.h"                    // Logging 
#include "human_face_detect.hpp"        // ML model
#include "bsp/esp-bsp.h"               
#include "esp_camera.h"                 // Camera driver           
#include "driver/gpio.h"                // GPIO control
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"              // for multitasking/ treading
#include "nvs_flash.h"                  // Non-volatile storage (NVS)
#include "esp_sleep.h"
#include "driver/adc.h"
#include <inttypes.h>

#define TAG "intermittent_face_detect"

// Power Monitoring Configuration
#define VOLTAGE_THRESHOLD_LOW 2.5   // Enter deep sleep below this (V)
#define VOLTAGE_THRESHOLD_OK 2.8    // Resume normal above this (V)
#define POWER_MONITOR_ADC_CHANNEL ADC1_CHANNEL_0
#define POWER_MONITOR_PIN GPIO_NUM_34

// Hardware Configuration
#define LED_GPIO GPIO_NUM_1

#define CAM_PIN_PWDN  GPIO_NUM_38   // Camera hardware pins
#define CAM_PIN_RESET -1
#define CAM_PIN_XCLK  GPIO_NUM_15
#define CAM_PIN_SIOD  GPIO_NUM_4
#define CAM_PIN_SIOC  GPIO_NUM_5
#define CAM_PIN_D7    GPIO_NUM_16
#define CAM_PIN_D6    GPIO_NUM_17
#define CAM_PIN_D5    GPIO_NUM_18
#define CAM_PIN_D4    GPIO_NUM_12
#define CAM_PIN_D3    GPIO_NUM_10
#define CAM_PIN_D2    GPIO_NUM_8
#define CAM_PIN_D1    GPIO_NUM_9
#define CAM_PIN_D0    GPIO_NUM_11
#define CAM_PIN_VSYNC GPIO_NUM_6
#define CAM_PIN_HREF  GPIO_NUM_7
#define CAM_PIN_PCLK  GPIO_NUM_13

// Checkpoint System
// state management system used for resuming executing
typedef enum {
    CHECKPOINT_INIT,
    CHECKPOINT_CAMERA_READY,
    CHECKPOINT_IMAGE_CAPTURED,
    CHECKPOINT_DETECTION_COMPLETE
} checkpoint_id_t;

typedef struct {
    // tracks application state across power cycles using NVS.
    uint32_t execution_count;   // Total power cycles (how many times device powered up)
    uint32_t face_detections;   // Cumulative faces found (how many faces detected)
    bool was_processing;        // Interrupted mid-prcoess
    uint32_t last_face_timestamp;
} app_state_t;

// Global State
// intializes global state
static app_state_t current_state = {
    .execution_count = 0,
    .face_detections = 0,
    .was_processing = false,
    .last_face_timestamp = 0
};
static TaskHandle_t mainTaskHandle = NULL;

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_DRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY
};

// Function Prototypes
static void IRAM_ATTR power_loss_isr(void* arg);        // ISR (Interrupt Service Routine) for power loss
static void init_power_monitoring();                    // sets up ADV to monitor voltage levels
static bool is_power_sufficient();                      
static bool is_power_critical();
static esp_err_t save_state();                          // saves current system state into NVS
static esp_err_t load_state();                          // loads saved states from NVS at boot
static void update_checkpoint(checkpoint_id_t checkpoint);
static checkpoint_id_t get_last_checkpoint();
static void enter_deep_sleep();
static void prepare_for_shutdown();
static esp_err_t init_camera();
static void process_face_detection(camera_fb_t *fb);

// Power Management
static void init_power_monitoring() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POWER_MONITOR_ADC_CHANNEL, ADC_ATTEN_DB_12);
    // GPIO interrupt setup for voltage drops

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << POWER_MONITOR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(POWER_MONITOR_PIN, power_loss_isr, NULL);
}

static bool is_power_sufficient() {
    int raw = adc1_get_raw(POWER_MONITOR_ADC_CHANNEL);
    float voltage = (raw * 3.3f) / 4095.0f;
    ESP_LOGD(TAG, "Voltage: %.2fV", voltage);
    return voltage > VOLTAGE_THRESHOLD_OK;
}

static bool is_power_critical() {
    int raw = adc1_get_raw(POWER_MONITOR_ADC_CHANNEL);
    float voltage = (raw * 3.3f) / 4095.0f;
    ESP_LOGD(TAG, "Voltage: %.2fV", voltage);
    return voltage < VOLTAGE_THRESHOLD_LOW;
}

// State Management
// uses NVS to persist state (current_state). Handles power failures and cold boots
static esp_err_t save_state() {
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("app_state", NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, "state", &current_state, sizeof(current_state)));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    ESP_LOGI(TAG, "State saved: executions=%" PRIu32 ", detections=%" PRIu32, 
             current_state.execution_count, current_state.face_detections);
    return ESP_OK;
}

static esp_err_t load_state() {
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("app_state", NVS_READWRITE, &handle));
    
    size_t size = sizeof(current_state);
    esp_err_t err = nvs_get_blob(handle, "state", &current_state, &size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        current_state = {0};
        ESP_LOGI(TAG, "No saved state found");
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "State loaded: executions=%" PRIu32 ", detections=%" PRIu32,
             current_state.execution_count, current_state.face_detections);
    return err;
}

static void update_checkpoint(checkpoint_id_t checkpoint) {
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open("app_state", NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_u8(handle, "checkpoint", checkpoint));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    ESP_LOGI(TAG, "Checkpoint updated: %d", checkpoint);
}

static checkpoint_id_t get_last_checkpoint() {
    nvs_handle_t handle;
    checkpoint_id_t checkpoint = CHECKPOINT_INIT;
    
    if (nvs_open("app_state", NVS_READONLY, &handle) == ESP_OK) {
        uint8_t value;
        if (nvs_get_u8(handle, "checkpoint", &value) == ESP_OK) {
            checkpoint = static_cast<checkpoint_id_t>(value);
        }
        nvs_close(handle);
    }
    return checkpoint;
}

// Interrupt Handlers
// interrupt service routine that notifies the main task to begin shutdown when power drops
static void IRAM_ATTR power_loss_isr(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mainTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

// System Operations
static void prepare_for_shutdown() {
    esp_camera_deinit();
    save_state();
    ESP_LOGI(TAG, "System ready for shutdown");
}

static void enter_deep_sleep() {
    ESP_LOGI(TAG, "Entering deep sleep");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_33, 1);
    esp_sleep_enable_timer_wakeup(30 * 1000000);
    gpio_set_level(LED_GPIO, 0);
    esp_deep_sleep_start();
}

// Camera Operations
static esp_err_t init_camera() {
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
    }
    return err;
}

static void process_face_detection(camera_fb_t *fb) {
    dl::image::jpeg_img_t jpeg_img = {
        .data = fb->buf,
        .width = static_cast<int>(fb->width),
        .height = static_cast<int>(fb->height),
        .data_size = fb->len
    };

    dl::image::img_t img;
    img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
    sw_decode_jpeg(jpeg_img, img, true);

    HumanFaceDetect detector;  // Detection threshold
    auto results = detector.run(img);

    current_state.face_detections += results.size();
    if (!results.empty()) {
        current_state.last_face_timestamp = esp_timer_get_time() / 1000000;
    }

    heap_caps_free(img.data);
    save_state();
}

// Main Application
extern "C" void app_main() {
    mainTaskHandle = xTaskGetCurrentTaskHandle();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    // Initialize hardware
    init_power_monitoring();
    load_state();

    // Configure LED GPIO
    gpio_config_t led_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_conf);

    // Main loop
    while (true) {
        if (is_power_critical()) {
            prepare_for_shutdown();
            enter_deep_sleep();
        }

        switch (get_last_checkpoint()) {
            case CHECKPOINT_INIT: {
                if (init_camera() == ESP_OK) {
                    update_checkpoint(CHECKPOINT_CAMERA_READY);
                }
                break;
            }

            case CHECKPOINT_CAMERA_READY: {
                gpio_set_level(LED_GPIO, 1);
                camera_fb_t *fb = esp_camera_fb_get();
                if (fb) {
                    update_checkpoint(CHECKPOINT_IMAGE_CAPTURED);
                    process_face_detection(fb);
                    esp_camera_fb_return(fb);
                    update_checkpoint(CHECKPOINT_DETECTION_COMPLETE);
                }
                gpio_set_level(LED_GPIO, 0);
                break;
            }

            case CHECKPOINT_IMAGE_CAPTURED:
                update_checkpoint(CHECKPOINT_CAMERA_READY);
                break;

            case CHECKPOINT_DETECTION_COMPLETE:
                ESP_LOGI(TAG, "Detection complete - %" PRIu32 " faces", current_state.face_detections);
                update_checkpoint(CHECKPOINT_CAMERA_READY);
                vTaskDelay(pdMS_TO_TICKS(2000));
                break;

            default:
                update_checkpoint(CHECKPOINT_INIT);
                break;
        }

        // Handle power loss interrupts
        if (ulTaskNotifyTake(pdTRUE, 0)) {
            prepare_for_shutdown();
            enter_deep_sleep();
        }

        // Periodic power check
        if (!is_power_sufficient()) {
            save_state();
            while (!is_power_sufficient() && !is_power_critical()) {
                vTaskDelay(pdMS_TO_TICKS(100));
            }
        }
    }
}
