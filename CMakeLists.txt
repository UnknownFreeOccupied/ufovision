add_library(Vision INTERFACE)
add_library(UFO::Vision ALIAS Vision)

target_link_libraries(Vision 
	INTERFACE 
		UFO::Math
)

set_target_properties(Vision 
	PROPERTIES
		INTERFACE_VERSION ${PROJECT_VERSION}
		INTERFACE_SOVERSION ${PROJECT_VERSION_MAJOR}
		INTERFACE_CXX_STANDARD 17
		INTERFACE_CXX_EXTENSIONS OFF
		OUTPUT_NAME "UFOVision"
)

include(GNUInstallDirs)

target_include_directories(Vision 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_TESTING)
  add_subdirectory(tests)
endif()

if(UFO_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Vision EXPORT Vision-targets
	COMPONENT Vision
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT Vision-targets
  FILE "Vision-targets.cmake"
  NAMESPACE UFO::
  DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Vision
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/Vision-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Vision-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Vision-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Vision-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Vision-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  COMPONENT Vision
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Vision
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)