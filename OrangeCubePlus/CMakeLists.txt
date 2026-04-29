# CMakeList.txt : CMake project for OrangeCubePlus, include source and define
# project specific logic here.
#

# Add source to this project's executable.
add_executable (OrangeCubePlus "OrangeCubePlus.cpp" "OrangeCubePlus.h")

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET OrangeCubePlus PROPERTY CXX_STANDARD 23)
endif()

# TODO: Add tests and install targets if needed.
target_link_libraries(OrangeCubePlus
	PRIVATE
	MAVSDK::mavsdk
)