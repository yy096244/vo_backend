project(vo_backend)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")

# find_package(OpenCV REQUIRED)
# find_package(Ceres REQUIRED)
# include_directories${CERES_INCLUDE_DIRS})
# target_link_libraries(${OpenCV_LIBS} ${CERES_LIBRARIES}) 

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# find_package(glog)

add_executable(vo_backend src/main.cpp)
target_link_libraries(vo_backend )