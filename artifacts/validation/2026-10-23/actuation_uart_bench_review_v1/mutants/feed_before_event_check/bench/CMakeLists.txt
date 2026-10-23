idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "." "../../portable" "../../firmware_smoke/main"
                       REQUIRES esp_timer esp_driver_uart esp_hw_support)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c++17 -Wall -Wextra -Werror
                       -fno-exceptions -fno-rtti -ffp-contract=off -fstack-usage)
