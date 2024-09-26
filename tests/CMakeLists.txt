include("${PROJECT_SOURCE_DIR}/3rdparty/catch2/catch2.cmake")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS    "--coverage")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
# # set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

add_executable(ufovision_tests
	color_test.cpp
)

target_link_libraries(ufovision_tests PRIVATE UFO::Vision Catch2::Catch2WithMain)

# target_compile_options(ufovision_tests
# 	PRIVATE
# 		-march=native
# )

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(ufovision_tests)