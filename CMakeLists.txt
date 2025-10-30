cmake_minimum_required(VERSION 3.22.1)
project(kite-motion-planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_COMPILER clang++) # more strict, better debugging

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

include_directories(src
	src/utils
	src/thirdparty
	src/thirdparty/bullet3/src
)

# Bullet

add_library(bullet
	src/thirdparty/bullet3/src/btLinearMathAll.cpp
	src/thirdparty/bullet3/src/btBulletCollisionAll.cpp
)
target_include_directories(bullet PRIVATE src/thirdparty/bullet3/src)
target_compile_definitions(bullet
	PUBLIC BT_EULER_DEFAULT_ZYX
	PUBLIC BT_USE_DOUBLE_PRECISION
)

# OMPL

find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
set( OMPL_DEPENDS
	${Boost_SERIALIZATION_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
	${Boost_SYSTEM_LIBRARY}
	${CMAKE_THREAD_LIBS_INIT}
)
configure_file(
	"src/thirdparty/ompl/config.h.in"
	"ompl/config.h"
)
file(GLOB_RECURSE OMPL_SOURCE
	"src/thirdparty/ompl/*.cpp"
)
list(FILTER OMPL_SOURCE EXCLUDE REGEX "src/thirdparty/ompl/extensions/.*\\.cpp")
add_library(ompl
	${OMPL_SOURCE}
)
target_include_directories(ompl PUBLIC
	"${CMAKE_CURRENT_BINARY_DIR}"
)
target_link_libraries(ompl ${OMPL_DEPENDS})

# Ruckig

file(GLOB_RECURSE RUCKIG_SOURCE
	"src/thirdparty/ruckig/src/*.cpp"
)
add_library(ruckig
	${RUCKIG_SOURCE}
)
target_include_directories(ruckig PUBLIC src/thirdparty/ruckig/include)

# planner

add_executable(main
	src/main.cpp
	src/interface/sceneio.cpp
	src/interface/planningrequest.cpp
	src/interface/preprocessor/converter.cpp
	src/collision/collisionchecker.cpp
	src/collision/shapes/shape.cpp
	src/collision/shapes/compoundshape.cpp
	src/collision/shapes/linkingcompoundshape.cpp
	src/collision/shapes/boxshape.cpp
	src/collision/shapes/cylinderzshape.cpp
	src/collision/shapes/inflatedconvexhullshape.cpp
	src/collision/shapes/visualmeshshape.cpp
	src/export/shapestojson.cpp
    src/interpolator/interpolator.cpp
	src/pathfinder/findpath.cpp
	src/pathfinder/RRTConnectCustom.cpp
)
target_compile_definitions(main
	PUBLIC COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
)

target_link_libraries(main bullet ompl ruckig)

add_executable(playground
	src/experiments/playground.cpp
	src/collision/collisionchecker.cpp
)
target_link_libraries(playground bullet)