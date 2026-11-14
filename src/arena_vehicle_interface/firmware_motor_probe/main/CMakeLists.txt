idf_component_register(SRCS "main.c" INCLUDE_DIRS "."
  REQUIRES esp_driver_uart esp_driver_gpio esp_driver_pcnt esp_driver_ledc esp_timer esp_system)
if(ARENA_MOTOR_GUARDED_5S OR ARENA_MOTOR_START15_5S)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ARENA_MOTOR_GUARDED_5S=1)
endif()
if(ARENA_MOTOR_START15_5S)
  target_compile_definitions(${COMPONENT_LIB} PRIVATE ARENA_MOTOR_START15_5S=1)
endif()
