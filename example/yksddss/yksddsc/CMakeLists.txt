#
# Copyright(c) 2020 ADLINK Technology Limited and others
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License v. 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
# v. 1.0 which is available at
# http://www.eclipse.org/org/documents/edl-v10.php.
#
# SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
#
cmake_minimum_required(VERSION 3.5)
project(yksdds LANGUAGES C CXX)

if(NOT TARGET CycloneDDS-CXX::ddscxx)
  find_package(CycloneDDS-CXX REQUIRED)
endif()

# 这个是调用生成器转换idl消息的，它自动能处理nubotmsg.hpp的依赖
# 选择手动转换idl消息，则需要处理较为复杂的依赖关系，因此直接使用生成器更为方便
idlcxx_generate(TARGET nubotddsmsg FILES nubotddsmsg.idl)

add_executable(z1body z1body.cpp)

# Link both executables to idl data type library and ddscxx.
target_link_libraries(z1body CycloneDDS-CXX::ddscxx nubotddsmsg)

# Disable the static analyzer in GCC to avoid crashing the GNU C++ compiler
# on Azure Pipelines
if(DEFINED ENV{SYSTEM_TEAMFOUNDATIONSERVERURI})
  if(CMAKE_C_COMPILER_ID STREQUAL "GNU" AND ANALYZER STREQUAL "on")
    target_compile_options(cxxpub PRIVATE -fno-analyzer)
    target_compile_options(cxxsub PRIVATE -fno-analyzer)
  endif()
endif()

set_property(TARGET z1body PROPERTY CXX_STANDARD 17)
