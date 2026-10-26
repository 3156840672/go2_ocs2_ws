cmake_minimum_required(VERSION 3.8)
project(qpoases_colcon)

find_package(ament_cmake REQUIRED)
include(FetchContent)

set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)
set(QPOASES_BUILD_EXAMPLES OFF CACHE BOOL "Disable qpOASES examples" FORCE)
set(QPOASES_DOWNLOAD_DIR "${CMAKE_CURRENT_BINARY_DIR}/download")

FetchContent_Declare(
  qpoasesDownload
  GIT_REPOSITORY https://github.com/coin-or/qpOASES.git
  GIT_TAG 9e40af7d170f440b7887fc4f9cf162f3f3ae24e8
  GIT_SHALLOW FALSE
  UPDATE_DISCONNECTED TRUE
  SOURCE_DIR "${QPOASES_DOWNLOAD_DIR}"
)
FetchContent_MakeAvailable(qpoasesDownload)

install(DIRECTORY "${qpoasesdownload_SOURCE_DIR}/include/" DESTINATION include)
install(TARGETS qpOASES
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES "${qpoasesdownload_SOURCE_DIR}/LICENSE.txt"
  DESTINATION share/${PROJECT_NAME}
  RENAME LICENSE.qpOASES)

ament_export_include_directories(include)
ament_export_libraries(qpOASES)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
