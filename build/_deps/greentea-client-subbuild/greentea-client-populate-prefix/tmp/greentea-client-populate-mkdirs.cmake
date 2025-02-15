# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-src")
  file(MAKE_DIRECTORY "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-src")
endif()
file(MAKE_DIRECTORY
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-build"
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix"
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/tmp"
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src"
  "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Users/alekp/Desktop/Auto_loco/STM32_Nucleo/Embedded_Platform/build/_deps/greentea-client-subbuild/greentea-client-populate-prefix/src/greentea-client-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
