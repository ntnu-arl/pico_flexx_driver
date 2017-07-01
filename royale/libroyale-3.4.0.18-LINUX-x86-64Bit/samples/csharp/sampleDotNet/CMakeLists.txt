cmake_minimum_required(VERSION 2.8)

project(sampleDotNet)

string(REGEX REPLACE "/" "\\\\" ROYALE_DOT_NET_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}) 

configure_file(
    "sampleDotNet.csproj.in"
    "sampleDotNet.csproj"
    )

include_external_msproject(sampleDotNet 
    ${CMAKE_CURRENT_BINARY_DIR}/sampleDotNet.csproj
    PLATFORM ${MACHINE}
    RoyaleDotNet)
   
set_target_properties(sampleDotNet
    PROPERTIES
    OUTPUT_NAME sampleDotNet
    FOLDER sampleDotNet
    )
