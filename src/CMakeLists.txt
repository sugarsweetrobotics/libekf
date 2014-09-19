include_directories(../include)
add_library(ekf ekf.cpp)

add_executable(ekf_demo demo.cpp)
target_link_libraries(ekf_demo ekf)


install(TARGETS ekf DESTINATION ${LIB_INSTALL_DIR}
        EXPORT libekf)
