cmake_minimum_required(VERSION 3.16)
project(radar_ekf LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Option 1: Try system Eigen3
find_package(Eigen3 3.3 QUIET NO_MODULE)

# Option 2: FetchContent fallback (downloads Eigen at configure time)
if(NOT Eigen3_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        eigen
        GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
        GIT_TAG 3.4.0
        GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(eigen)
    # Eigen is header-only, create imported target for consistency
    if(NOT TARGET Eigen3::Eigen)
        add_library(Eigen3::Eigen INTERFACE IMPORTED)
        set_target_properties(Eigen3::Eigen PROPERTIES
            INTERFACE_INCLUDE_DIRECTORIES "${eigen_SOURCE_DIR}"
        )
    endif()
endif()

add_library(radar_ekf STATIC
    src/radar_ekf.cpp
)

target_include_directories(radar_ekf PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(radar_ekf PUBLIC Eigen3::Eigen)

# Install
install(TARGETS radar_ekf
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# Examples
add_executable(radar_ekf_example examples/example.cpp)
target_link_libraries(radar_ekf_example PRIVATE radar_ekf)

# Tests
enable_testing()
add_executable(radar_ekf_test tests/test.cpp)
target_link_libraries(radar_ekf_test PRIVATE radar_ekf)
add_test(NAME radar_ekf_test COMMAND radar_ekf_test)