cmake_minimum_required( VERSION 3.12 )

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

project( msl-raptor-backend )
set( INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include" )

set( CMAKE_CXX_STACK_SIZE "10000000" )
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules" )

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release )
endif()

set( PROJ_NAME "SR-UKF" )
set( OUTPUT_NAME "msl-raptor-example" )

include( CheckCXXCompilerFlag )
CHECK_CXX_COMPILER_FLAG( "-std=c++11" COMPILER_SUPPORTS_CXX11 )
CHECK_CXX_COMPILER_FLAG( "-std=c++0x" COMPILER_SUPPORTS_CXX0X )

if( COMPILER_SUPPORTS_CXX11 )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" )
elseif( COMPILER_SUPPORTS_CXX0X )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x" )
else()
    message( STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler." )
endif()

if ( CMAKE_COMPILER_IS_GNUCXX )
    set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated -Wno-deprecated-declarations -Wno-attributes -O3 -g" )
endif ( CMAKE_COMPILER_IS_GNUCXX )

if ( NOT DEFINED EIGEN3_INCLUDE_DIRS )
    message( STATUS "You may want to specify EIGEN3_INCLUDE_DIRS: cmake .. -DEIGEN3_INCLUDE_DIRS='eigen3/path'" )
    FIND_PACKAGE( Eigen3 REQUIRED )
endif ()


find_package(OpenCV 3.0 QUIET)
if(NOT OpenCV_FOUND)
  find_package(OpenCV_FOUND)
  if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0 QUIET)
    if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
    endif()
  endif()
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()



message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")



message(STATUS "Eigen library status:")
message(STATUS "    version: ${Eigen3_VERSION}")
message(STATUS "    libraries: ${Eigen3_LIBS}")
message(STATUS "    include path: ${Eigen3_INCLUDE_DIRS}")


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${INCLUDE_DIR}
  )

set(
  SOURCES
  msl-raptor-example.cpp
)

set(
  HEADERS
  include/UKF.h
  include/msl_raptor_backend.h
)

add_executable( ${PROJ_NAME} ${SOURCES} ${HEADERS} )
set_target_properties( ${PROJ_NAME} PROPERTIES OUTPUT_NAME ${OUTPUT_NAME} )
# target_link_libraries(main ${OpenCV_LIBS})

if ( MSVC )
    set_property(TARGET ${PROJ_NAME} APPEND PROPERTY LINK_FLAGS /DEBUG)
else ()
    target_link_libraries( ${PROJ_NAME} -pthread ${OpenCV_LIBS} OpenMP::OpenMP_CXX)
endif ( MSVC )

# Create source group for headers
source_group( "Header Files" FILES ${HEADERS} )
