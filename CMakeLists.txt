######################################################################## 
 # Build Soapy SDR support module for TimsSDR
 ######################################################################## 

 cmake_minimum_required(VERSION 3.22)
 project(SoapyTimsSDR CXX)
 
 find_package(SoapySDR "0.7" NO_MODULE)
  if (NOT SoapySDR_FOUND) 
     message(FATAL_ERROR "Soapy SDR development files not found...") 
  endif () 
 
 list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
 find_package(LIBTIMSSDR)
 
 if (NOT LIBTIMSSDR_FOUND) 
      message(FATAL_ERROR "TimsSDR development files not found...") 
 endif () 
 message(STATUS "LIBTIMSSDR_INCLUDE_DIRS - ${LIBTIMSSDR_INCLUDE_DIRS}")
 message(STATUS "LIBTIMSSDR_LIBRARIES - ${LIBTIMSSDR_LIBRARIES}")
 
 #version check for recent timssdr with device list API
 message(STATUS "Checking for timssdr_device_list API...")
 message(STATUS "  Reading ${LIBTIMSSDR_INCLUDE_DIRS}/timssdr.h...")
 file(READ ${LIBTIMSSDR_INCLUDE_DIRS}/timssdr.h timssdr_h)
 string(FIND "${timssdr_h}" "timssdr_device_list" has_timssdr_device_list)
 if ("${has_timssdr_device_list}" STREQUAL "-1")
     message(FATAL_ERROR "  libtimssdr too old, missing timssdr_device_list API")
 endif()
 
 include_directories(${CMAKE_CURRENT_SOURCE_DIR})
 include_directories(${LIBTIMSSDR_INCLUDE_DIRS})
 
 #enable c++11 features
 if(CMAKE_COMPILER_IS_GNUCXX)
 
     #C++11 is a required language feature for this project
     include(CheckCXXCompilerFlag)
     CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_STD_CXX11)
     if(HAS_STD_CXX11)
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
     else(HAS_STD_CXX11)
         set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
     endif()
 
     #disable warnings for unused parameters
     add_definitions(-Wno-unused-parameter)
 
 endif(CMAKE_COMPILER_IS_GNUCXX)
 
 if (APPLE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wc++11-extensions")
 endif(APPLE)
 
 SOAPY_SDR_MODULE_UTIL(
     TARGET TimsSDRSupport
     SOURCES
     TimsSDR_Registration.cpp
     TimsSDR_Settings.cpp
     TimsSDR_Streaming.cpp
     TimsSDR_Session.cpp
     LIBRARIES ${LIBTIMSSDR_LIBRARIES}
 )
 
 target_link_libraries(TimsSDRSupport usb)

 ########################################################################
 # uninstall target
 ########################################################################
 add_custom_target(uninstall
     "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
 configure_file(
     "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
     "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
     IMMEDIATE @ONLY)
 
 