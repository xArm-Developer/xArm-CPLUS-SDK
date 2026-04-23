cmake_minimum_required(VERSION 3.5)

file(READ "${CMAKE_CURRENT_SOURCE_DIR}/include/xarm/wrapper/xarm_api.h" XARM_API_HEADER_TEXT)
string(REGEX MATCH "#define[ \t]+SDK_VERSION[ \t]+\"([0-9]+\\.[0-9]+\\.[0-9]+)\"" _sdk_version_match "${XARM_API_HEADER_TEXT}")
if(CMAKE_MATCH_1)
  set(XARM_SDK_VERSION "${CMAKE_MATCH_1}")
else()
  set(XARM_SDK_VERSION "0.0.0")
endif()

project(xarm_cplus_sdk VERSION ${XARM_SDK_VERSION} LANGUAGES CXX)

option(BUILD_SHARED_LIBS "Build xarm as a shared library" ON)
option(XARM_BUILD_EXAMPLES "Build all example executables" ON)
option(XARM_BUILD_CSHARP_WRAPPER "Build the C# wrapper library on Windows" OFF)
option(XARM_ENABLE_INSTALL "Enable install rules" ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(GNUInstallDirs)

set(XARM_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")
set(XARM_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src")
set(XARM_EXAMPLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/example")

set(XARM_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set(XARM_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${XARM_LIBRARY_OUTPUT_DIRECTORY}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${XARM_LIBRARY_OUTPUT_DIRECTORY}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${XARM_RUNTIME_OUTPUT_DIRECTORY}")

if(MSVC)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_compile_options(/W3)
else()
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    -Wno-sign-compare
    -Wno-unused-parameter
    -Wno-unused-variable
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wno-unused-but-set-variable -Wno-type-limits)
  endif()
endif()

file(GLOB XARM_SERIAL_COMMON_SOURCES
  "${XARM_SRC_DIR}/serial/*.cc"
)
file(GLOB XARM_CORE_COMMON_SOURCES
  "${XARM_SRC_DIR}/xarm/core/common/*.cc"
)
file(GLOB XARM_CORE_UTILS_SOURCES
  "${XARM_SRC_DIR}/xarm/core/utils/*.cc"
)
file(GLOB XARM_CORE_INSTRUCTION_SOURCES
  "${XARM_SRC_DIR}/xarm/core/instruction/*.cc"
)
file(GLOB XARM_CORE_OS_SOURCES
  "${XARM_SRC_DIR}/xarm/core/os/*.cc"
)
file(GLOB XARM_CORE_PORT_SOURCES
  "${XARM_SRC_DIR}/xarm/core/port/*.cc"
)
file(GLOB XARM_CORE_TOP_SOURCES
  "${XARM_SRC_DIR}/xarm/core/*.cc"
)
file(GLOB XARM_WRAPPER_SOURCES
  "${XARM_SRC_DIR}/xarm/wrapper/*.cc"
)

set(XARM_PLATFORM_SOURCES)
if(WIN32)
  list(APPEND XARM_PLATFORM_SOURCES
    "${XARM_SRC_DIR}/serial/impl/win.cc"
    "${XARM_SRC_DIR}/serial/impl/list_ports/list_ports_win.cc"
  )
elseif(APPLE)
  list(APPEND XARM_PLATFORM_SOURCES
    "${XARM_SRC_DIR}/serial/impl/unix.cc"
    "${XARM_SRC_DIR}/serial/impl/list_ports/list_ports_osx.cc"
  )
else()
  list(APPEND XARM_PLATFORM_SOURCES
    "${XARM_SRC_DIR}/serial/impl/unix.cc"
    "${XARM_SRC_DIR}/serial/impl/list_ports/list_ports_linux.cc"
  )
endif()

set(XARM_SOURCES
  ${XARM_SERIAL_COMMON_SOURCES}
  ${XARM_PLATFORM_SOURCES}
  ${XARM_CORE_COMMON_SOURCES}
  ${XARM_CORE_UTILS_SOURCES}
  ${XARM_CORE_INSTRUCTION_SOURCES}
  ${XARM_CORE_OS_SOURCES}
  ${XARM_CORE_PORT_SOURCES}
  ${XARM_CORE_TOP_SOURCES}
  ${XARM_WRAPPER_SOURCES}
)

add_library(xarm ${XARM_SOURCES})
set_target_properties(xarm PROPERTIES
  OUTPUT_NAME xarm
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)

if(WIN32 AND BUILD_SHARED_LIBS)
  set_target_properties(xarm PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

target_include_directories(xarm PUBLIC "${XARM_INCLUDE_DIR}")

find_package(Threads REQUIRED)
target_link_libraries(xarm PUBLIC Threads::Threads)

if(WIN32)
  target_link_libraries(xarm PRIVATE ws2_32)
elseif(APPLE)
  find_library(IOKIT_FRAMEWORK IOKit REQUIRED)
  find_library(COREFOUNDATION_FRAMEWORK CoreFoundation REQUIRED)
  target_link_libraries(xarm PRIVATE ${IOKIT_FRAMEWORK} ${COREFOUNDATION_FRAMEWORK})
else()
  target_link_libraries(xarm PRIVATE m)
endif()

if(XARM_BUILD_EXAMPLES)
  file(GLOB XARM_EXAMPLE_SOURCES
    "${XARM_EXAMPLE_DIR}/*.cc"
  )

  foreach(example_source IN LISTS XARM_EXAMPLE_SOURCES)
    get_filename_component(example_name "${example_source}" NAME_WE)
    add_executable("${example_name}" "${example_source}")
    target_link_libraries("${example_name}" PRIVATE xarm)
    target_include_directories("${example_name}" PRIVATE "${XARM_INCLUDE_DIR}")
  endforeach()
endif()

if(WIN32 AND XARM_BUILD_CSHARP_WRAPPER)
  add_library(xarm_wrapper SHARED
    "${CMAKE_CURRENT_SOURCE_DIR}/demo/csharp_studio/xarm/xarm_wrapper.cc"
  )
  set_target_properties(xarm_wrapper PROPERTIES
    OUTPUT_NAME xarm_wrapper
    RUNTIME_OUTPUT_DIRECTORY "${XARM_RUNTIME_OUTPUT_DIRECTORY}"
    LIBRARY_OUTPUT_DIRECTORY "${XARM_LIBRARY_OUTPUT_DIRECTORY}"
    ARCHIVE_OUTPUT_DIRECTORY "${XARM_LIBRARY_OUTPUT_DIRECTORY}"
  )
  target_include_directories(xarm_wrapper PRIVATE
    "${XARM_INCLUDE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/demo/csharp_studio/xarm"
  )
  target_link_libraries(xarm_wrapper PRIVATE xarm)
endif()

if(XARM_ENABLE_INSTALL)
  install(TARGETS xarm
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  )

  install(DIRECTORY "${XARM_INCLUDE_DIR}/serial"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  )
  install(DIRECTORY "${XARM_INCLUDE_DIR}/xarm"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
  )
endif()

message(STATUS "xArm SDK version: ${PROJECT_VERSION}")
message(STATUS "Build shared libs: ${BUILD_SHARED_LIBS}")
message(STATUS "Build examples: ${XARM_BUILD_EXAMPLES}")
message(STATUS "Build C# wrapper: ${XARM_BUILD_CSHARP_WRAPPER}")
