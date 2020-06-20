cmake_minimum_required (VERSION 3.1.0)
project(planning)

set(CMAKE_CXX_FLAGS "-std=c++1y -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)
find_package(OpenRAVE 0.9 REQUIRED)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
    add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
#find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)
find_package(CGAL REQUIRED)
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT  COMPONENTS iostreams python thread system)

include_directories(${OpenRAVE_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS} ${CGAL_INCLUDE_DIRS} include)
link_directories(${OpenRAVE_LIBRARY_DIRS}  ${Boost_LIBRARY_DIRS} ${CGAL_LIBRARY_DIRS})
add_library(planning SHARED src/torsoplanning_A*.cpp)
set_target_properties(planning PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(planning ${OpenRAVE_LIBRARIES} ${CGAL_LIBRARIES})
install(TARGETS planning DESTINATION .)
#file(GLOB SOURCES "src/*.cpp")
#add_executable(testmain ${SOURCES})
#set_target_properties(testmain PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
#target_link_libraries(testmain ${OpenRAVE_LIBRARIES} ${OpenRAVE_CORE_LIBRARIES} ${Boost_THREAD_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${CGAL_LIBRARIES} )
