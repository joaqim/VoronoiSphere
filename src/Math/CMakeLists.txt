cmake_minimum_required(VERSION 3.5.1)

if (POLICY CMP0048)
  cmake_policy(SET CMP0048 NEW)
endif (POLICY CMP0048)

project(TerrificMath VERSION 1 LANGUAGES CXX)

set(TerrificMath_SRCS
  Math.cpp
  )
set(TerrificMath_HEADERS
  Math.hpp
  Math.h
  Types.h
  xs_Config.h
  xs_Float.h
  )


add_library(TerrificMathObjects OBJECT
	${TerrificMath_SRCS}
	${TerrificMath_HEADERS})
target_include_directories(TerrificMathObjects PUBLIC
	${JSON_INCLUDE_DIR}
	../../
	$<TARGET_PROPERTY:Terrific,INTERFACE_INCLUDE_DIRECTORIES>
	)

set_target_properties(TerrificMathObjects PROPERTIES FOLDER "Terrific/Math")
set_target_properties(TerrificMathObjects PROPERTIES COMPILE_FLAGS "-fPIC")

add_library(TerrificMath ${SHARED_OR_STATIC}
	$<TARGET_OBJECTS:TerrificMathObjects>)
set_target_properties(TerrificMath PROPERTIES COMPILE_FLAGS "-fPIC")
set_target_properties(TerrificMath PROPERTIES 
	DEBUG_POSTFIX "-d" 
	FOLDER "Terrific/Math")

target_link_libraries(TerrificMath Terrific)

install(TARGETS TerrificMath
	RUNTIME DESTINATION ${TERRIFIC_BINARY_INSTALL_DIR}
	LIBRARY DESTINATION ${TERRIFIC_LIBRARY_INSTALL_DIR}
	ARCHIVE DESTINATION ${TERRIFIC_LIBRARY_INSTALL_DIR})

install(FILES ${TerrificMath_HEADERS} DESTINATION ${TERRIFIC_INCLUDE_INSTALL_DIR}/Math)

add_library(Terrific::Math ALIAS TerrificMath)

if(BUILD_TESTS)
  #	add_subdirectory(Test)
endif()


