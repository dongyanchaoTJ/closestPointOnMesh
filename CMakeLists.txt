cmake_minimum_required(VERSION 2.6)
project(closestpoint)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_C_FLAGS ${CMAKE_C_FLAGS_RELEASE})
SET(EXE_NAME ${PROJECT_NAME})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_SOURCE_DIR}/bin)

INCLUDE_DIRECTORIES(
               ${CMAKE_CURRENT_SOURCE_DIR} 
			   include
			   )	   

AUX_SOURCE_DIRECTORY(${CMAKE_CURRENT_SOURCE_DIR}/src source_files)
SET(dir ${CMAKE_CURRENT_SOURCE_DIR}/src)
file( GLOB header_files "${dir}/*.h" )
SET(solution ${source_files} ${header_files})
ADD_EXECUTABLE(${EXE_NAME} ${solution})
