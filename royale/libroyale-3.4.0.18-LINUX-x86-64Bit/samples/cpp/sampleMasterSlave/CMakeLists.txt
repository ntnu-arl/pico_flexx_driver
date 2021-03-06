cmake_minimum_required(VERSION 2.8)

set(CMAKE_PREFIX_PATH "../../../share")

find_package(royale REQUIRED)

link_directories(${royale_LIB_DIR})

project (sampleMasterSlave)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../inc
)

add_executable(sampleMasterSlave
   sampleMasterSlave.cpp
   )

if (WIN32)
    add_custom_command(
        TARGET sampleMasterSlave
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/royale.dll"  $<TARGET_FILE_DIR:sampleMasterSlave>)
endif (WIN32)

if (APPLE)
    add_custom_command(
        TARGET sampleMasterSlave
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.dylib"  $<TARGET_FILE_DIR:sampleMasterSlave>)

    add_custom_command(
        TARGET sampleMasterSlave
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.${royale_VERSION}.dylib" $<TARGET_FILE_DIR:sampleMasterSlave>)

    add_custom_command(
        TARGET sampleMasterSlave
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libuvc.dylib" $<TARGET_FILE_DIR:sampleMasterSlave>)
endif (APPLE)

target_link_libraries(sampleMasterSlave "${royale_LIBS}")
