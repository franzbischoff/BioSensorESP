FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(
  SRCS ${app_sources}
  REQUIRES "freertos" "esp_adc" "spi_flash"
)

list(APPEND compile_options "-fsanitize=undefined" "-fno-sanitize=shift-base")
