# # set(src_dirs        ./)

# # set(include_dirs    ./)

# # set(requires        human_face_detect)

# # if (IDF_TARGET STREQUAL "esp32s3")
# #     list(APPEND requires esp32_s3_eye_noglib
# #                          esp_lcd)
# # elseif (IDF_TARGET STREQUAL "esp32p4")
# #     list(APPEND requires esp32_p4_function_ev_board_noglib
# #                          esp_lcd)
# # endif()

# # set(embed_files     "human_face.jpg")

# # idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires} EMBED_FILES ${embed_files})

# set(src_dirs        ./)
# set(include_dirs    ./)

# # Core requirements
# set(requires        human_face_detect
#                     nvs_flash
#                     esp32-camera
#                     log
#                     driver)

# # Conditional dependencies based on target
# if (IDF_TARGET STREQUAL "esp32s3")
#     list(APPEND requires esp32_s3_eye_noglib esp_lcd)
# elseif (IDF_TARGET STREQUAL "esp32p4")
#     list(APPEND requires esp32_p4_function_ev_board_noglib esp_lcd)
# endif()

# # Embedded resources
# set(embed_files     "human_face.jpg")

# # Register the component
# idf_component_register(SRC_DIRS ${src_dirs}
#                        INCLUDE_DIRS ${include_dirs}
#                        REQUIRES ${requires}
#                        EMBED_FILES ${embed_files})

set(src_dirs        .)
set(include_dirs    .)

# Required components
set(requires
    human_face_detect
    esp-dl
    esp32-camera
    nvs_flash
    driver
    esp_adc
    esp_timer
    esp_common
    esp_rom
    esp_system
    esp_hw_support
    esp_psram
    esp_pm
    freertos
    hal
    log
    soc
    heap
    esp_http_client
    esp_https_ota
    sdmmc
    spi_flash
)

# Conditional board-specific components
if (IDF_TARGET STREQUAL "esp32s3")
    list(APPEND requires esp32_s3_eye_noglib)
elseif (IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp32_p4_function_ev_board_noglib)
endif()

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
    EMBED_FILES "human_face.jpg"
)