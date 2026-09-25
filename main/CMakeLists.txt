idf_component_register(
    SRCS "main.c" "usb.c" "ingest.c" "pipeline.c" "ai.cpp" "hud.c" "stats.c"
         "led.c" "lpcore.c" "inet.c" "web.c"
    INCLUDE_DIRS "."
    REQUIRES esp_tinyusb tinyusb esp_netif esp_event lwip esp_http_server app_update nvs_flash
             esp_driver_usb_serial_jtag esp_driver_jpeg esp_driver_ppa esp_driver_tsens esp_driver_rmt esp_timer esp_mm
             esp_psram mbedtls esp_h264 led_strip ulp esp-dl human_face_detect cat_detect
             pedestrian_detect
    EMBED_TXTFILES "dashboard.html")

# Programa del núcleo LP (ulp/main.c) -> lp_core_main.bin embebido en la app
set(ulp_app_name lp_core_main)
set(ulp_lp_core_sources "ulp/main.c")
set(ulp_exp_dep_srcs "lpcore.c")
ulp_embed_binary(${ulp_app_name} "${ulp_lp_core_sources}" "${ulp_exp_dep_srcs}")
