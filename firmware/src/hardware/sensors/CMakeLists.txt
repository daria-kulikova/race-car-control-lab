idf_component_register(
    SRCS
        "adc.c"
        "bmi088.c"
        "sensors.c"

        "bosch_sensortec/bmi08g.c"
        "bosch_sensortec/bmi08a.c"
    INCLUDE_DIRS
        "include"
        "bosch_sensortec"
    REQUIRES
        control-types
        spi-master
        "esp_adc_cal"
)
