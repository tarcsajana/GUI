# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options("-Wno-attributes")

# Keep OTA token out of source control by reading it from local environment.
if(DEFINED ENV{GUI_GITHUB_TOKEN} AND NOT "$ENV{GUI_GITHUB_TOKEN}" STREQUAL "")
	add_compile_definitions(GUI_GITHUB_TOKEN="$ENV{GUI_GITHUB_TOKEN}")
endif()

project(lvgl_porting)
