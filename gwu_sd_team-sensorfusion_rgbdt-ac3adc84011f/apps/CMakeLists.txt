# 2D Viewer of thermal stream
message("-- Generating build for ThermalViewer2D. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ThermalViewer2D)

message("-- Generating build for ThermalCalibration. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ThermalCalibration)

message("-- Generating build for ThermalSensorAnalysis. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ThermalSensorAnalysis)

message("-- Generating build for Viewer3D. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(Viewer3D)

message("-- Generating build for Viewer5D. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(Viewer5D)

message("-- Generating build for ViewerFusion. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ViewerFusion)

message("-- Generating build for DatasetPublisher. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(DatasetPublisher)

message("-- Generating build for DatasetSubscriber. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(DatasetSubscriber)

message("-- Generating build for DatasetRegistration. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(DatasetRegistration)

message("-- Generating build for DatasetPlayer. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(DatasetPlayer)

message("-- Generating build for ViewerSkeleton. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ViewerSkeleton)

message("-- Generating build for ViewerKeyPatch. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ViewerKeyPatch)

message("-- Generating build for ThermalModelPublisher. Host: ${CMAKE_SYSTEM_NAME}")
add_subdirectory(ThermalModelPublisher)
