<<<<<<< HEAD
 HEAD
[supported]: https://img.shields.io/badge/-supported-green "supported"

| Chip     | ESP-IDF v5.3           | ESP-IDF v5.4           |
|----------|------------------------|------------------------|
| ESP32-S3 | ![alt text][supported] | ![alt text][supported] |
| ESP32-P4 | ![alt text][supported] | ![alt text][supported] |

#Intermittent Face Detection on ESP32-S3

[![supported](https://img.shields.io/badge/-supported-green)]()

This project demonstrates **intermittent machine learning** on the ESP32-S3, using a camera and face detection model powered entirely by a solar panel and supercapacitor. It features:

- Low-power design with deep sleep transitions
- Real-time face detection using ESP-DL's `HumanFaceDetect` model
- Power-aware sensing via ADC voltage thresholds
- Persistent state management using NVS
- Safe shutdown and resumption using checkpointing

## Quick start

Follow the [quick start](https://docs.espressif.com/projects/esp-dl/en/latest/getting_started/readme.html#quick-start) to flash the example, you will see the output in idf monitor:

```
I (955) human_face_detect: [score: 0.936285, x1: 100, y1: 64, x2: 193, y2: 191]
I (955) human_face_detect: left_eye: [117, 114], left_mouth: [120, 160], nose: [132, 143], right_eye: [157, 112], right_mouth: [151, 160]]
```

## Configurable Options in Menuconfig

### Component configuration
We provide the models as components, each of them has some configurable options. See [Human Face Detect Model](https://github.com/espressif/esp-dl/blob/master/models/human_face_detect/README.md)。

### Project configuration

- CONFIG_PARTITION_TABLE_CUSTOM_FILENAME

If model location is set to FLASH partition, please set this option to `partitions2.csv`

# esp_intermittent_face_ml
 daf093bd5ab2254d4746752e7b71c4289d36ff23
=======
# esp_intermittent_face_ml
>>>>>>> daf093bd5ab2254d4746752e7b71c4289d36ff23
