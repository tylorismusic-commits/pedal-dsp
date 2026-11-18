cmake_minimum_required(VERSION 3.16)
project(pedal_dsp VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(RUBBERBAND REQUIRED IMPORTED_TARGET rubberband)

find_path(HDSP_INCLUDE_DIR hdsp/db.hpp
  HINTS
    ${CMAKE_CURRENT_SOURCE_DIR}/../hearth-dsp/include
    ${CMAKE_CURRENT_SOURCE_DIR}/third_party/hearth-dsp/include
    /usr/local/include
  REQUIRED)

add_library(pedal_dsp STATIC src/drive808.cpp src/pitch.cpp)
target_include_directories(pedal_dsp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(pedal_dsp PUBLIC ${HDSP_INCLUDE_DIR})
target_include_directories(pedal_dsp PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/guitarix/plugins)
target_link_libraries(pedal_dsp PUBLIC PkgConfig::RUBBERBAND)
target_compile_features(pedal_dsp PUBLIC cxx_std_17)

include(GNUInstallDirs)
install(DIRECTORY include/stomp DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS pedal_dsp EXPORT pedal_dspTargets)
install(EXPORT pedal_dspTargets
  NAMESPACE pedal_dsp::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/pedal_dsp)

option(PEDAL_DSP_BUILD_TESTS "Build pedal-dsp tests" ON)
if(PEDAL_DSP_BUILD_TESTS)
  enable_testing()
  add_executable(pedal_dsp_test tests/test_modules.cpp)
  target_link_libraries(pedal_dsp_test PRIVATE pedal_dsp)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(pedal_dsp_test PRIVATE -Wall -Wextra)
  endif()
  add_test(NAME pedal_dsp COMMAND pedal_dsp_test)
endif()
