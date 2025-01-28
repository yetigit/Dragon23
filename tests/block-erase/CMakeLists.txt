
cmake_minimum_required(VERSION 3.9.1)


get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" PROJECT_NAME ${PROJECT_NAME})
project(${PROJECT_NAME})

set(DRGN_REL_ROOT "../..")
set(DRAGON_INCLUDE_DIR "${DRGN_REL_ROOT}/src/include")
set(EIGEN_INCLUDE_DIR "${DRGN_REL_ROOT}/external/eigen")
set(ROBIN_HOOD_INCLUDE_DIR "${DRGN_REL_ROOT}/external/robin-hood-hashing/src/include")
set(MIMA_DIR "${DRGN_REL_ROOT}/external/mimalloc")
set(MIMA_INC "${MIMA_DIR}/include")

set(CMAKE_CXX_STANDARD 17)

set (FMT_INCL "${DRGN_REL_ROOT}/external/fmt/include") 
# set (GTEST_INC "${DRGN_REL_ROOT}/external/googletest/googletest/include") 
add_executable(
    ${PROJECT_NAME} "${PROJECT_NAME}.cpp"
                    ${DRAGON_HEADERS})


target_include_directories(
    ${PROJECT_NAME} PUBLIC ${DRAGON_INCLUDE_DIR} ${EIGEN_INCLUDE_DIR}
                            ${ROBIN_HOOD_INCLUDE_DIR} ${FMT_INCL})
target_link_libraries(${PROJECT_NAME} PUBLIC mimalloc gtest gtest_main)

 
if (WIN32)
target_compile_definitions(${PROJECT_NAME} PRIVATE -DWIN32 -DFMT_HEADER_ONLY -D_USE_MATH_DEFINES -DTINYOBJLOADER_IMPLEMENTATION)
target_compile_options(${PROJECT_NAME} PUBLIC -Zi)
else()

target_compile_definitions(${PROJECT_NAME} PRIVATE -DFMT_HEADER_ONLY -D_USE_MATH_DEFINES -DTINYOBJLOADER_IMPLEMENTATION)
endif()

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

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
        "${CMAKE_BINARY_DIR}/external/mimalloc/$<CONFIGURATION>/libmimalloc.dylib"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>) 
endif()