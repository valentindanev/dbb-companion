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
        db_diag.c
        db_fc_flash.c
        db_fc_params.c
        db_fc_tune.c
        db_netlog.c
        sonar_driver.c
        sonar_dyp_l041mtw.c
        sonar_aj_sr04m.c
        db_ota_policy.c
        dbb_brain_stub.c
)
set(COMPONENT_INCLUDE_DIRS ".")

if(CONFIG_WEB_DEPLOY_EMBEDDED)
    set(WEB_GENERATED_DIR "${CMAKE_BINARY_DIR}/generated/web")
    file(MAKE_DIRECTORY "${WEB_GENERATED_DIR}")
    set(WEB_GENERATED_C "${WEB_GENERATED_DIR}/dbb_web_assets.c")
    set(WEB_GENERATED_H "${WEB_GENERATED_DIR}/dbb_web_assets.h")
    # File-level deps are required: "DEPENDS frontend" alone is only an order-only target
    # dependency, so edits to the web sources would never re-run embed_web_assets.py.
    # (New frontend files still need one reconfigure to enter the glob.)
    file(GLOB DBB_WEB_SOURCE_FILES
            "${CMAKE_SOURCE_DIR}/frontend/*.js"
            "${CMAKE_SOURCE_DIR}/frontend/*.html"
            "${CMAKE_SOURCE_DIR}/frontend/*.css"
            "${CMAKE_SOURCE_DIR}/frontend/*.png"
            "${CMAKE_SOURCE_DIR}/frontend/*.ico")
    add_custom_command(
            OUTPUT "${WEB_GENERATED_C}" "${WEB_GENERATED_H}"
            COMMAND ${CMAKE_COMMAND} -E make_directory "${WEB_GENERATED_DIR}"
            COMMAND ${PYTHON} "${CMAKE_SOURCE_DIR}/tools/embed_web_assets.py"
                    --input "${CMAKE_BINARY_DIR}/frontend"
                    --output-c "${WEB_GENERATED_C}"
                    --output-h "${WEB_GENERATED_H}"
            DEPENDS frontend "${CMAKE_SOURCE_DIR}/tools/embed_web_assets.py"
                    ${DBB_WEB_SOURCE_FILES}
            VERBATIM)
    set_source_files_properties("${WEB_GENERATED_C}" PROPERTIES GENERATED TRUE)
    list(APPEND SRCS "${WEB_GENERATED_C}")
    list(APPEND COMPONENT_INCLUDE_DIRS "${WEB_GENERATED_DIR}")
endif()

idf_component_register(SRCS ${SRCS}
        INCLUDE_DIRS ${COMPONENT_INCLUDE_DIRS})


if(CONFIG_WEB_DEPLOY_SF)
    set(WEB_SRC_DIR "${CMAKE_BINARY_DIR}/frontend")
    spiffs_create_partition_image(www ${WEB_SRC_DIR} FLASH_IN_PROJECT DEPENDS frontend)
endif()
