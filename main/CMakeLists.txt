set(srcs
    main.c
    speech_commands_action.c
    )

set(requires
    hardware_driver
    esp32-camera
    esp_wifi
    nvs_flash
    esp-tls
    esp_http_client
    esp_lcd
    esp_adc
    )

idf_component_register(SRCS ${srcs}
                       INCLUDE_DIRS include
                       REQUIRES ${requires}
                       EMBED_TXTFILES telegram_certificate.pem)

component_compile_options(-w)