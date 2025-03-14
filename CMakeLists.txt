cmake_minimum_required(VERSION 3.10)
project(AT4VG)

set(CMAKE_CXX_STANDARD 20)

if (${DEVICE} MATCHES "vs1000p|vs2000|vs2000p")
    set(OpenCV_DIR "/opt/libs_aarch64/opencv_aarch64/lib/cmake/opencv4")
elseif (${DEVICE} MATCHES "vs800|vg800")
    set(OpenCV_DIR "/opt/libs_aarch64/mier_opencv/lib/cmake/opencv4")
elseif (${DEVICE} MATCHES "vs600|vs800p")
    set(OpenCV_DIR "/opt/libs_aarch32/opencv-4.5.2-contrib-linux-aarch32/lib/cmake/opencv4")
endif ()
find_package(OpenCV REQUIRED PATHS)

add_compile_definitions(USE_V4L2)

add_library(AT SHARED "${CMAKE_SOURCE_DIR}/src/at_interface.cpp")
target_include_directories(AT PUBLIC "${CMAKE_SOURCE_DIR}/include")
target_link_libraries(AT PUBLIC ${OpenCV_LIBS})

set(LibCap_DIR "${CMAKE_SOURCE_DIR}/test/libcap/${DEVICE}")
file(GLOB LibCap_LIBS "${LibCap_DIR}/lib/*.a" "${LibCap_DIR}/lib/*.so")
add_executable(at4vg_server "${CMAKE_SOURCE_DIR}/test/server_test/server.cpp")
target_link_libraries(at4vg_server PUBLIC
        AT
        ${LibCap_LIBS}
        ${OpenCV_LIBS}
)
target_include_directories(at4vg_server PUBLIC
        "${LibCap_DIR}/inc"
        "${CMAKE_SOURCE_DIR}/include"
)

set(INSTALL_DIR "${CMAKE_SOURCE_DIR}/release/${DEVICE}")
install(FILES "${CMAKE_SOURCE_DIR}/include/at_interface.h" DESTINATION "${INSTALL_DIR}/include")
install(TARGETS AT DESTINATION "${INSTALL_DIR}/lib")
install(TARGETS at4vg_server DESTINATION "${INSTALL_DIR}/bin")
