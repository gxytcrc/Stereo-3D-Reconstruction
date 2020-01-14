cmake_minimum_required(VERSION 2.8)

set(CMAKE_BUILD_TYPE Release)
set(DCMAKE_BUILD_TYPE Release)

project(3dReconstruct)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/libelas/src)
include_directories(${PROJECT_SOURCE_DIR}/VO/src)

FILE(GLOB VO_SRC_FILES "VO/src/*.cpp")


add_executable(3dReconstruct ${PROJECT_SOURCE_DIR}/src/main.cpp ${PROJECT_SOURCE_DIR}/src/stereoDepth.cpp ${PROJECT_SOURCE_DIR}/src/pointCloudMapping.cpp ${PROJECT_SOURCE_DIR}/src/vo.cpp ${PROJECT_SOURCE_DIR}/libelas/src/descriptor.cpp ${PROJECT_SOURCE_DIR}/libelas/src/elas.cpp ${PROJECT_SOURCE_DIR}/libelas/src/filter.cpp ${PROJECT_SOURCE_DIR}/libelas/src/matrix.cpp ${PROJECT_SOURCE_DIR}/libelas/src/triangle.cpp ${VO_SRC_FILES})

target_link_libraries(3dReconstruct ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})

