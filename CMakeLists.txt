cmake_minimum_required(VERSION 2.8.3)
project(dahua_stereo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
  message_runtime
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}  
  main.cpp  
  CamHandlerNode.cpp )

add_dependencies(${PROJECT_NAME}  include)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES} 
  -lrt 
  -lpthread
  /opt/DahuaTech/MVviewer/lib/  
  /opt/DahuaTech/MVviewer/lib/libImageConvert.so
  /opt/DahuaTech/MVviewer/lib/libMVSDK.so
  /opt/DahuaTech/MVviewer/lib/libVideoRender.so
  -I/usr/include/opencv -lopencv_shape -lopencv_stitching -lopencv_superres -lopencv_videostab -lopencv_aruco -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_datasets -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_line_descriptor -lopencv_optflow -lopencv_video -lopencv_plot -lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_rgbd -lopencv_viz -lopencv_surface_matching -lopencv_text -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core
  )
