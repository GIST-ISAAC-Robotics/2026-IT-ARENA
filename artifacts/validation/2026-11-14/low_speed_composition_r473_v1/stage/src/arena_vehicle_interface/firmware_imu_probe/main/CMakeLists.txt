set(ST_DRIVER_DIR "$ENV{ST_DRIVER_DIR}")
if(NOT ST_DRIVER_DIR)
  message(FATAL_ERROR "ST_DRIVER_DIR must point to pinned lsm6dsox-pid source")
endif()
idf_component_register(SRCS "main.c" "${ST_DRIVER_DIR}/lsm6dsox_reg.c"
  INCLUDE_DIRS "${ST_DRIVER_DIR}" REQUIRES esp_driver_spi esp_driver_uart esp_driver_gpio esp_timer esp_psram spi_flash esp_system)
