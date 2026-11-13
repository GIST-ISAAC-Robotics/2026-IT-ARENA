set(ST_DRIVER_DIR "$ENV{ST_DRIVER_DIR}")
if(NOT ST_DRIVER_DIR)
  message(FATAL_ERROR "ST_DRIVER_DIR must point to pinned lsm6dsox-pid source")
endif()
idf_component_register(SRCS "main.cpp" "../../firmware_runtime_preview/main/imu_source.c"
  "${ST_DRIVER_DIR}/lsm6dsox_reg.c"
  INCLUDE_DIRS "${ST_DRIVER_DIR}" "../../portable" "../../firmware_runtime_preview/main"
  REQUIRES esp_driver_spi esp_driver_uart esp_driver_gpio esp_driver_pcnt esp_driver_ledc
    esp_timer esp_psram spi_flash esp_system)
target_compile_options(${COMPONENT_LIB} PRIVATE -fstack-usage)
option(ARENA_RUNTIME_INSTALLED_PROFILE "Explicit installed profile in this fresh build copy" OFF)
if(ARENA_RUNTIME_INSTALLED_PROFILE)
  if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../runtime_installed_profile/runtime_installed_profile.hpp")
    message(FATAL_ERROR "Explicit installed profile is missing from the build copy")
  endif()
  target_include_directories(${COMPONENT_LIB} PRIVATE "../../runtime_installed_profile")
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ARENA_RUNTIME_INSTALLED_PROFILE=1)
endif()
