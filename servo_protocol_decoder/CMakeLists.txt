# Copyright 2026 Gerardo Puga
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.8)
project(servo_protocol_decoder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Fetch cppcrc library
include(FetchContent)
fetchcontent_declare(
  cppcrc
  GIT_REPOSITORY https://github.com/DarrenLevine/cppcrc.git
  GIT_TAG 6360eee1c8966d32b2552ce156b135ec6a3235f1
)
fetchcontent_makeavailable(cppcrc)

# Library
add_library(${PROJECT_NAME} SHARED
  src/servo_protocol_decoder.cpp
  src/crc16.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${cppcrc_SOURCE_DIR}
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_crc16
    test/test_crc16.cpp
  )

  target_link_libraries(test_crc16
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_packet_buffer_builder
    test/test_packet_buffer_builder.cpp
  )

  target_link_libraries(test_packet_buffer_builder
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_servo_protocol_decoder
    test/test_servo_protocol_decoder.cpp
  )

  target_link_libraries(test_servo_protocol_decoder
    ${PROJECT_NAME}
  )

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
