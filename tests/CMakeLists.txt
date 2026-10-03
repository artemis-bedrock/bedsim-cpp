include(FetchContent)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
set(BUILD_GMOCK OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
	googletest
	URL https://github.com/google/googletest/archive/refs/tags/v1.17.0.zip
	SYSTEM
)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

file(GLOB bedsim_test_sources CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

foreach(source ${bedsim_test_sources})
	get_filename_component(name ${source} NAME_WE)
	add_executable(bedsim_${name}_tests ${source})
	target_link_libraries(bedsim_${name}_tests PRIVATE bedsim::bedsim GTest::gtest_main)
	bedsim_enable_warnings(bedsim_${name}_tests)
	gtest_discover_tests(bedsim_${name}_tests DISCOVERY_MODE PRE_TEST)
endforeach()
