project(TestKinematics)

#boost
find_package(Boost COMPONENTS system thread filesystem)

#Eigen
set(EIGEN_DIR ${EIGEN_DIR} CACHE PATH "The directory of the Eigen library")
set(EIGEN_INCLUDE_DIRS ${EIGEN_DIR})

include_directories(
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${EIGEN_INCLUDE_DIRS}
)

link_directories(
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
)

add_executable(TestKinematics ForwardKinematics.cpp Link.cpp testKinematics.cpp ForwardKinematics.h Link.h LinkParameter.h)

target_link_libraries(TestKinematics ${Boost_LIBRARIES})
