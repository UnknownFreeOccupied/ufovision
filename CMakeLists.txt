cmake_minimum_required(VERSION 3.5...3.16)

project(ufovision
	VERSION 2.0.0
	DESCRIPTION "The UFO vision library"
	LANGUAGES CXX
)

add_subdirectory(1stparty)

add_library(ufovision INTERFACE)
add_library(UFO::Vision ALIAS ufovision)

set_target_properties(ufovision PROPERTIES
	VERSION ${PROJECT_VERSION}
	SOVERSION ${PROJECT_VERSION_MAJOR}
	CXX_STANDARD 17
	CXX_EXTENSIONS OFF
)

target_include_directories(ufovision INTERFACE
	$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

target_compile_features(ufovision INTERFACE cxx_std_17)

target_link_libraries(ufovision INTERFACE UFO::Math)

include(GNUInstallDirs)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Only enable tests when we're the root project
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  add_subdirectory(tests)
endif()

install(TARGETS ufovision
	EXPORT ufovisionTargets
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(EXPORT ufovisionTargets
	FILE "ufovisionTargets.cmake"
	NAMESPACE UFO::
	DESTINATION lib/cmake/${PROJECT_NAME}
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
	"${PROJECT_SOURCE_DIR}/cmake/ufovisionConfig.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/ufovisionConfig.cmake"
	INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/ufovisionConfigVersion.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

INSTALL(
	FILES
	"${CMAKE_CURRENT_BINARY_DIR}/ufovisionConfig.cmake"
	"${CMAKE_CURRENT_BINARY_DIR}/ufovisionConfigVersion.cmake"
	DESTINATION lib/cmake/${PROJECT_NAME}
)

install(
	DIRECTORY ${PROJECT_SOURCE_DIR}/include/
	DESTINATION include
)
