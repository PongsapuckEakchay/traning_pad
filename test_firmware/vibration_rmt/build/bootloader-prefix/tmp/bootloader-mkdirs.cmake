# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/ai/esp/esp-idf/components/bootloader/subproject"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/tmp"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/src"
  "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/ai/Documents/IWING/iwing101/test_firmware/vibration_rmt/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
