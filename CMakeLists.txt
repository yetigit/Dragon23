cmake_minimum_required(VERSION 3.9.1)

if(APPLE)
message(WARNING "Configuring build for MAC")
endif()

set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_STANDARD 17)


set(PROJECT_NAME Dragon)

set(DRAGON_EXAMPLES
    OFF
    CACHE BOOL "Build the examples projects")

set(DRAGON_TESTS
    OFF
    CACHE BOOL "Build tests")

set(DRAGON_EXECUTABLE
    OFF
    CACHE BOOL "Build Dragon.exe")

set(DRAGON_DOCUMENTATION
    OFF
    CACHE BOOL "Generates some documentation")
set(MI_BUILD_TESTS
    OFF
    CACHE BOOL " ")



    
if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choose the type of build, options are: [None, Debug, Release, RelWithDebInfo, MinSizeRel]"
       FORCE )
endif()

project(${PROJECT_NAME})

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(DRAGON_INCLUDE_DIR "src/include")
set(EIGEN_INCLUDE_DIR "external/eigen")
set(ROBIN_HOOD_INCLUDE_DIR "external/robin-hood-hashing/src/include")

set(OSD_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/OpenSubdiv")

file(GLOB_RECURSE DRAGON_HEADERS "src/include/*.hpp")



if(${DRAGON_TESTS})
    set(BUILD_GMOCK OFF CACHE BOOL "")
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/googletest")
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/tests")
endif()

if(${DRAGON_EXAMPLES})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/examples")
endif()

if(${DRAGON_DOCUMENTATION})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/docs")
endif()

if(${DRAGON_EXECUTABLE})
    add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/external/mimalloc")
    set (FMT_INCL "${CMAKE_CURRENT_SOURCE_DIR}/external/fmt/include") 
    add_executable(
    ${PROJECT_NAME} "${CMAKE_CURRENT_SOURCE_DIR}/examples/main.cpp"
                    ${DRAGON_HEADERS})

	target_precompile_headers(${PROJECT_NAME} PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/include/GnMesh/Geom/GnmEigen.PCH.hpp") 
	set(CMAKE_PCH_INSTANTIATE_TEMPLATES ON)

    target_include_directories(
    ${PROJECT_NAME} PRIVATE ${DRAGON_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR}
                            ${ROBIN_HOOD_INCLUDE_DIR} ${FMT_INCL})
    target_link_libraries(${PROJECT_NAME} PUBLIC mimalloc)

 
if (APPLE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE -DFMT_HEADER_ONLY -D_USE_MATH_DEFINES -DTINYOBJLOADER_IMPLEMENTATION)
else()

    target_compile_definitions(${PROJECT_NAME} PRIVATE -DWIN32 -DFMT_HEADER_ONLY -D_USE_MATH_DEFINES -DTINYOBJLOADER_IMPLEMENTATION)
endif()

if (WIN32)
    target_compile_options(${PROJECT_NAME} PUBLIC -Zi)
    target_link_options(${PROJECT_NAME} PUBLIC -DEBUG)
endif()

if(${DRAGON_EXECUTABLE})
    if (WIN32)
    # c:\repos\Dragon\build\external\mimalloc\Debug\
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${CMAKE_BINARY_DIR}/external/mimalloc/$<CONFIGURATION>/mimalloc.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${CMAKE_BINARY_DIR}/external/mimalloc/$<CONFIGURATION>/mimalloc-redirect.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:${PROJECT_NAME}>  )
    else()


    # c:\repos\Dragon\build\external\mimalloc\Debug\
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${CMAKE_BINARY_DIR}/external/mimalloc/$<CONFIGURATION>/libmimalloc.dylib"      # <--this is in-file
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
          )
    endif()
endif()

endif()
