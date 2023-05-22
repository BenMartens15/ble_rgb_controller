idf_component_register(SRCS "led.c" "ble.c" "main.c" "rgb.c"
                    INCLUDE_DIRS ".")
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
