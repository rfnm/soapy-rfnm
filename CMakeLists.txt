#
# CMake
#

cmake_minimum_required(VERSION 3.22)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

INCLUDE(FindPkgConfig)

# Install to PyBOMBS target prefix if defined
if(DEFINED ENV{PYBOMBS_PREFIX})
    set(CMAKE_INSTALL_PREFIX $ENV{PYBOMBS_PREFIX})
    set(PYBOMBS_PREFIX $ENV{PYBOMBS_PREFIX})
    message(STATUS "PyBOMBS installed GNU Radio. Setting CMAKE_INSTALL_PREFIX to $ENV{PYBOMBS_PREFIX}")
endif()

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
if (librfnm_ADDED)
  include_directories("${librfnm_SOURCE_DIR}/include")
endif()

#include(cmake/cpm-fmtlib.cmake)
include(cmake/cpm-spdlog.cmake)
include(cmake/cpm-soapysdr.cmake)

find_package(SoapySDR)
if(NOT SOAPYSDR_FOUND)
  message("Building SoapySDR")
  include(cmake/cpm-soapysdr.cmake)
  set(SOAPYSDR_LIBRARIES SoapySDR)
else()
  message("Found installed SoapySDR")
  include_directories(${SOAPYSDR_INCLUDE_DIRS})
endif()

#
# Library
#

add_library(soapy-rfnm SHARED)

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
#target_link_libraries(soapy-rfnm PRIVATE fmt::fmt)

target_link_libraries(soapy-rfnm PRIVATE 
 ${SOAPYSDR_LIBRARIES}
 spdlog 
 rfnm
)


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
