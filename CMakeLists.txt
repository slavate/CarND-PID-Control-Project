project(PID)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources src/PID.cpp src/main.cpp)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

if (WIN32)
    set(VCPKG_INSTALL ${_VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET})
    message("Using vcpkg_install ${VCPKG_INSTALL}")
    include_directories(${VCPKG_INSTALL}/include)
    link_directories(${VCPKG_INSTALL}/lib)
endif()

add_executable(pid ${sources})

if (NOT WIN32)
    target_link_libraries(pid z ssl uv uWS)
else()
    target_link_libraries(pid ssleay32 libuv uWS ws2_32)
    target_link_libraries(pid optimized zlib )
	target_link_libraries(pid debug zlibd )
endif()
