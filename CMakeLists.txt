cmake_minimum_required(VERSION 3.28)
project(kuka_iiwa)
find_package(drake CONFIG REQUIRED)
find_package(ompl REQUIRED)
add_executable(Test src/kuka_iiwa.cc)
include_directories(
	${OMPL_INCLUDE_DIRS}
)
target_link_libraries(Test drake::drake ${OMPL_LIBRARIES})
