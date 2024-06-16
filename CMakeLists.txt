#
# CMake
#

cmake_minimum_required(VERSION 3.22)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#
# Project
#

project(soapy-rfnm
  LANGUAGES CXX
  VERSION 0.0.1
)

#select the release build type by default to get optimization flags
if(NOT DEFINED CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
  message(STATUS "Build type not specified: defaulting to release.")
endif()

#enable PIC globally
set(CMAKE_POSITION_INDEPENDENT_CODE ON)






#
# Dependencies
#


#find_package(librfnm REQUIRED)
#set(librfnm_DIR "../librfnm")


include(cmake/cpm.cmake)
include(cmake/cpm-librfnm.cmake)
#include(cmake/cpm-fmtlib.cmake)
include(cmake/cpm-spdlog.cmake)
include(cmake/cpm-soapysdr.cmake)

#
# Library
#

add_library(soapy-rfnm SHARED)

target_link_libraries(soapy-rfnm PRIVATE rfnm)

target_compile_options(soapy-rfnm PRIVATE -Wall -Wextra -Wno-unused-parameter)

# sources
target_sources(soapy-rfnm PRIVATE
  "src/soapy_rfnm.cpp"
)

# definitions
if(MSVC)
  target_compile_definitions(soapy-rfnm PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# features
target_compile_features(soapy-rfnm PUBLIC cxx_std_23)

# libraries
#target_link_libraries(soapy-rfnm PRIVATE LIBUSB::LIBUSB)
target_link_libraries(soapy-rfnm PRIVATE SoapySDR)
#target_link_libraries(soapy-rfnm PRIVATE fmt::fmt)
target_link_libraries(soapy-rfnm PRIVATE spdlog)
target_link_libraries(soapy-rfnm PRIVATE rfnm)

#target_include_directories(soapy-rfnm PUBLIC "../librfnm/librfnm.h")

#include_directories("${CMAKE_SOURCE_DIR}/librfnm")


#
# Installation
#

set(DEST_DIR ${CMAKE_INSTALL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION}/)

install(
  FILES "$<TARGET_RUNTIME_DLLS:soapy-rfnm>"
  DESTINATION ${DEST_DIR}
)

install(
  TARGETS soapy-rfnm
  DESTINATION ${DEST_DIR}
)

#if(MSVC)
#  install(
#    FILES "$<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:$<TARGET_PDB_FILE:soapy-rfnm>>"
#    DESTINATION "bin"
#  )
#endif()
