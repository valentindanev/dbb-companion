# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.10)

add_custom_target(
        frontend
        # add final and temporary build directory for the frontend. That way npm does not need to know the CMAKE_BINARY_DIR which can variable
        COMMAND cmake -E make_directory "${CMAKE_BINARY_DIR}/frontend"
        COMMAND cmake -E make_directory "${CMAKE_SOURCE_DIR}/frontend/build"
        # Run frontend npm build process
        COMMAND npm install && npm run build
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/frontend")
# Copy the content of npm to the build directory
add_custom_command(
        TARGET frontend POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_SOURCE_DIR}/frontend/build"
        "${CMAKE_BINARY_DIR}/frontend")
# delete npm temp build folder
add_custom_command (
        TARGET frontend
        POST_BUILD
        COMMAND cmake -E remove_directory "${CMAKE_SOURCE_DIR}/frontend/build"
)
set(PROJECT_VER "1.0.1")

# --- DBB Companion: optional private brain -------------------------------
# If the private brain folder exists next door, pull it into the build so its
# strong dbb_brain_* symbols override the open base's weak stubs. For the
# community this folder is absent -> the open base builds & runs standalone.
if(EXISTS "${CMAKE_SOURCE_DIR}/../dbb-brain-private/components")
    set(EXTRA_COMPONENT_DIRS "${CMAKE_SOURCE_DIR}/../dbb-brain-private/components")
    message(STATUS "DBB Companion: private brain FOUND -> building FULL product")
else()
    message(STATUS "DBB Companion: no private brain -> building OPEN base (weak stubs)")
endif()
# ------------------------------------------------------------------------

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(db_esp32)
