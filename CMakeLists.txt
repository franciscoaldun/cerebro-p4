# p4-webcam ("Cerebro P4") — webcam con IA en el ESP32-P4 (WT9932P4-TINY, chip v1.0).
# La imagen entra por USB (cámara del PC o, más adelante, cámara MIPI), el P4 corre la IA
# y la devuelve como webcam UVC + red NCM con dashboard.
cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# Clase de video (UVC) de TinyUSB: esp_tinyusb no la trae en su configuración, se activa aquí
idf_build_set_property(COMPILE_DEFINITIONS "CFG_TUD_VIDEO=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CFG_TUD_VIDEO_STREAMING=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE=32768" APPEND)
project(p4_webcam)
