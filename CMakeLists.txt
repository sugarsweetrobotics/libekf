cmake_minimum_required(VERSION 2.8)
project(ekf)


if(WIN32 AND NOT CYGWIN)
  set(INC_INSTALL_DIR "/ekf/include")
  set(LIB_INSTALL_DIR "/ekf/lib")
else()
  set(INC_INSTALL_DIR "/usr/local/include")
  set(LIB_INSTALL_DIR "/usr/local/lib")
endif()

include_directories(include)

add_subdirectory(src)

install(DIRECTORY include/
   DESTINATION ${INC_INSTALL_DIR}
   FILES_MATCHING PATTERN "*.h"
                  PATTERN "*.hxx"
   PATTERN ".svn" EXCLUDE
)
install(
   EXPORT libekf DESTINATION ${LIB_INSTALL_DIR}
)
