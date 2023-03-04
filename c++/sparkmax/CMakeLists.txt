cmake_minimum_required(VERSION 3.5)
project(sparkmax)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#file(GLOB_RECURSE includedSrc CONFIGURE_DEPENDS "*.h" "*.cpp")

include_directories(
	include
	/usr/local/include
)

add_executable(${PROJECT_NAME} 
	src/sparkmax.cpp
	#${includedSrc}
)

target_link_libraries(${PROJECT_NAME}
	libwpilibc.so
	libwpiutil.so
	libwpimath.so
	libntcore.so
	libwpiHal.so
	libREVLib.so
	libREVLibDriver.so
)


