// #include "esp_log.h"
// #include "human_face_detect.hpp"
// #include "bsp/esp-bsp.h"

// extern const uint8_t human_face_jpg_start[] asm("_binary_human_face_jpg_start");
// extern const uint8_t human_face_jpg_end[] asm("_binary_human_face_jpg_end");
// const char *TAG = "human_face_detect";

// extern "C" void app_main(void)
// {
// #if CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD
//     ESP_ERROR_CHECK(bsp_sdcard_mount());
// #endif

//     dl::image::jpeg_img_t jpeg_img = {.data = (uint8_t *)human_face_jpg_start,
//                                       .width = 320,
//                                       .height = 240,
//                                       .data_size = (uint32_t)(human_face_jpg_end - human_face_jpg_start)};
//     dl::image::img_t img;
//     img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
//     sw_decode_jpeg(jpeg_img, img, true);

//     HumanFaceDetect *detect = new HumanFaceDetect();

//     auto &detect_results = detect->run(img);
//     for (const auto &res : detect_results) {
//         ESP_LOGI(TAG,
//                  "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
//                  res.score,
//                  res.box[0],
//                  res.box[1],
//                  res.box[2],
//                  res.box[3]);
//         ESP_LOGI(
//             TAG,
//             "left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], right_eye: [%d, %d], right_mouth: [%d, %d]]",
//             res.keypoint[0],
//             res.keypoint[1],
//             res.keypoint[2],
//             res.keypoint[3],
//             res.keypoint[4],
//             res.keypoint[5],
//             res.keypoint[6],
//             res.keypoint[7],
//             res.keypoint[8],
//             res.keypoint[9]);
//     }
//     delete detect;
//     heap_caps_free(img.data);

// #if CONFIG_HUMAN_FACE_DETECT_MODEL_IN_SDCARD
//     ESP_ERROR_CHECK(bsp_sdcard_unmount());
// #endif
// }

#include "esp_log.h"
#include "human_face_detect.hpp"
#include "bsp/esp-bsp.h"
#include "esp_camera.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#define TAG "human_face_detect_loop"

// GPIO for onboard LED (Freenove ESP32-S3 WROOM)
#define LED_GPIO GPIO_NUM_1

// Camera pin definitions for ESP32S3 WROOM
#define CAM_PIN_PWDN  GPIO_NUM_38
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

static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
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
    //.sccb_i2c_port = I2C_NUM_0
};

extern "C" void app_main(void)
{
    esp_err_t err;
    ESP_LOGI(TAG, "Initializing NVS and camera");

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    // Init LED GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Init camera
    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return;
    }

    while (true) {
        // Flash LED on
        gpio_set_level(LED_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_GPIO, 0);

        ESP_LOGI(TAG, "Capturing image");
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Failed to capture image");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        dl::image::jpeg_img_t jpeg_img = {
            .data = fb->buf,
            .width = (int)fb->width,
            .height =(int)fb->height,
            .data_size = fb->len
        };

        dl::image::img_t img;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;
        sw_decode_jpeg(jpeg_img, img, true);

        HumanFaceDetect *detect = new HumanFaceDetect();
        auto &results = detect->run(img);

        for (const auto &res : results) {
            ESP_LOGI(TAG, "[score: %f, x1: %d, y1: %d, x2: %d, y2: %d]",
                     res.score, res.box[0], res.box[1], res.box[2], res.box[3]);
            ESP_LOGI(TAG, "left_eye: [%d, %d], left_mouth: [%d, %d], nose: [%d, %d], right_eye: [%d, %d], right_mouth: [%d, %d]",
                     res.keypoint[0], res.keypoint[1], res.keypoint[2], res.keypoint[3],
                     res.keypoint[4], res.keypoint[5], res.keypoint[6], res.keypoint[7],
                     res.keypoint[8], res.keypoint[9]);
        }

        delete detect;
        heap_caps_free(img.data);
        esp_camera_fb_return(fb);

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}
