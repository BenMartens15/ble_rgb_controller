idf_component_register(SRCS "ble_core.c" "rgb_controller.c" "led.c" "main.c"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
