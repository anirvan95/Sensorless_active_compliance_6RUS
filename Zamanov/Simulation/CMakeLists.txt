PROJECT (RBDLEXAMPLE CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )

# Search for the RBDL include directory and library
FIND_PACKAGE (RBDL COMPONENTS LuaModel REQUIRED)
FIND_PACKAGE (Eigen3 3.0.0 REQUIRED)
FIND_PACKAGE (Lua51 REQUIRED)
#FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)

# Add the include directory to the include paths
INCLUDE_DIRECTORIES ( ${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${LUA_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR} )

ADD_LIBRARY(RUS6 RUS6.cpp)

# Create an executable
ADD_EXECUTABLE (zamanov_mechanism Main.cc)

# And link the library against the executable
TARGET_LINK_LIBRARIES ( zamanov_mechanism
	RUS6
	${RBDL_LIBRARY}
	${RBDL_LuaModel_LIBRARY}
	${LUA_LIBRARIES}
	)
