SET( viewdemo_SRC
  "pcldemo_neareast.cpp"
)
include_directories("/usr/local/include/pcl-1.0" "/usr/include/vtk-5.2" "/usr/include/eigen3")
ADD_EXECUTABLE( pcldemo ${viewdemo_SRC} )
TARGET_LINK_LIBRARIES( pcldemo NUKinect "-lpcl_visualization" "-lpcl_segmentation")
