idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "." "../../portable"
                       REQUIRES esp_timer)
target_compile_options(${COMPONENT_LIB} PRIVATE -std=c++17 -Wall -Wextra -Werror
                       -fno-exceptions -fno-rtti -ffp-contract=off -fstack-usage)
