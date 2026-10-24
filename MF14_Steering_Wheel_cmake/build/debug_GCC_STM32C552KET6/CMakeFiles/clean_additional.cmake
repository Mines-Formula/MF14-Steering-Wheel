# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "debug_GCC_STM32C552KET6")
  file(REMOVE_RECURSE
  "MF14_Steering_Wheel.elf"
  "MF14_Steering_Wheel.map"
  )
endif()
