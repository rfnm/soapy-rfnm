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
include(cmake/cpm-asio.cmake)

set(LIBRFNM_LOCAL_PATH "${CMAKE_SOURCE_DIR}/../librfnm" CACHE PATH "Path to local librfnm source")
if(EXISTS "${LIBRFNM_LOCAL_PATH}/CMakeLists.txt")
    message(STATUS "Using local librfnm at: ${LIBRFNM_LOCAL_PATH}")
    add_subdirectory("${LIBRFNM_LOCAL_PATH}" "${CMAKE_BINARY_DIR}/librfnm-build")
    include_directories("${LIBRFNM_LOCAL_PATH}/include")
else()
    include(cmake/cpm-librfnm.cmake)
    if (librfnm_ADDED)
      include_directories("${librfnm_SOURCE_DIR}/include")
    endif()
endif()




#include(cmake/cpm-fmtlib.cmake)
#include(cmake/cpm-spdlog.cmake)
#include_directories("${spdlog_SOURCE_DIR}/include")

add_compile_definitions(SPDLOG_FMT_EXTERNAL=0)

# Find spdlog
find_package(spdlog CONFIG REQUIRED)

include_directories(${asio_SOURCE_DIR}/asio/include)

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

# Set the output name for the module to follow SoapySDR naming convention
set_target_properties(soapy-rfnm PROPERTIES 
    OUTPUT_NAME "RFNMSupport"
    PREFIX ""  # Remove lib prefix on Unix-like systems
)

if(NOT MSVC)
    target_compile_options(soapy-rfnm PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

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
 spdlog::spdlog
 rfnm_static
)

#
# Installation
#
# Query SoapySDR for the module installation directory
if(SOAPYSDR_FOUND)
    # First, try to get the module directory from SoapySDR CMake variables
    if(DEFINED SOAPY_SDR_MODULE_DIR)
        set(SOAPY_MODULE_DIR ${SOAPY_SDR_MODULE_DIR})
        message(STATUS "Using SoapySDR provided module directory: ${SOAPY_MODULE_DIR}")
    
    # Check if SoapySDRConfig provides the install paths
    elseif(COMMAND SOAPY_SDR_MODULE_UTIL)
        # If SOAPY_SDR_MODULE_UTIL is available, use it instead of manual installation
        # This is the preferred method for SoapySDR modules
        SOAPY_SDR_MODULE_UTIL(
            TARGET soapy-rfnm
            SOURCES src/soapy_rfnm.cpp
            LIBRARIES
                spdlog::spdlog
                rfnm_static
        )
        # Skip manual installation since SOAPY_SDR_MODULE_UTIL handles it
        return()
    
    # Try to extract from SoapySDR's install configuration  
    elseif(TARGET SoapySDR)
        # Modern CMake: try to get the install location from the imported target
        get_target_property(SOAPY_LIB_LOCATION SoapySDR LOCATION)
        if(SOAPY_LIB_LOCATION)
            get_filename_component(SOAPY_LIB_DIR ${SOAPY_LIB_LOCATION} DIRECTORY)
            get_filename_component(SOAPY_ROOT_DIR ${SOAPY_LIB_DIR} DIRECTORY)
            set(SOAPY_MODULE_DIR ${SOAPY_ROOT_DIR}/lib/SoapySDR/modules${SOAPY_SDR_ABI_VERSION})
            message(STATUS "Derived SoapySDR module directory from library location: ${SOAPY_MODULE_DIR}")
        endif()
    endif()
endif()

# Fallback: use CMAKE_INSTALL_PREFIX with standard layout
if(NOT DEFINED SOAPY_MODULE_DIR)
    include(GNUInstallDirs)
    
    # Ensure we have the ABI version
    if(NOT DEFINED SOAPY_SDR_ABI_VERSION)
        # Try to get it from SoapySDR
        if(DEFINED SoapySDR_VERSION)
            string(REGEX MATCH "^([0-9]+\\.[0-9]+)" SOAPY_SDR_ABI_VERSION "${SoapySDR_VERSION}")
        else()
            # Default to 0.8 which is common for recent versions
            set(SOAPY_SDR_ABI_VERSION "0.8")
            message(WARNING "Could not determine SoapySDR ABI version, defaulting to ${SOAPY_SDR_ABI_VERSION}")
        endif()
    endif()
    
    # Construct the module directory path
    set(SOAPY_MODULE_DIR ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI_VERSION})
    message(STATUS "Using fallback SoapySDR module directory: ${SOAPY_MODULE_DIR}")
endif()

# Final installation path
set(DEST_DIR ${SOAPY_MODULE_DIR})
message(STATUS "SoapySDR modules will be installed to: ${DEST_DIR}")



# Install the module
install(
    FILES "$<TARGET_RUNTIME_DLLS:soapy-rfnm>"
    DESTINATION ${DEST_DIR}
)
install(
    TARGETS soapy-rfnm
    DESTINATION ${DEST_DIR}
)

# Optional: Install debug files on Windows for development
if(MSVC AND CMAKE_BUILD_TYPE MATCHES "Debug|RelWithDebInfo")
    install(
        FILES "$<TARGET_PDB_FILE:soapy-rfnm>"
        DESTINATION ${DEST_DIR}
        OPTIONAL
    )
endif()