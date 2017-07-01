cmake_minimum_required(VERSION 2.8)

project (royale_samples)

# C++ samples
add_subdirectory(cpp/sampleCameraInfo)
add_subdirectory(cpp/sampleRetrieveData)
add_subdirectory(cpp/sampleOpenCV)
add_subdirectory(cpp/sampleMasterSlave)
add_subdirectory(cpp/sampleExportPLY)

# C sample
add_subdirectory(c/sampleCAPI)

# C# sample
if(MSVC)
    add_subdirectory(csharp/sampleDotNet)
endif()
