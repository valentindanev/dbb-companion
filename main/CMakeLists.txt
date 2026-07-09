set(SRCS
        main.c
        tcp_server.c
        http_server.c
        db_esp32_control.c
        db_serial.c
        db_mavlink_msgs.c
        db_parameters.c
        db_timers.c
        danevi_sonar.c
        deeper_udp_sonar.c
        db_sonar_log.c
        dbb_brain_stub.c
)

idf_component_register(SRCS ${SRCS}
        INCLUDE_DIRS ".")


if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_BINARY_DIR}/frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT DEPENDS frontend)
endif()
