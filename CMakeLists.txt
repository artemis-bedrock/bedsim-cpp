cmake_minimum_required(VERSION 3.30)

project(bedsim LANGUAGES CXX)

option(BEDSIM_BUILD_TESTS "Build bedsim tests" ${PROJECT_IS_TOP_LEVEL})

function(bedsim_enable_warnings target)
	if(MSVC)
		target_compile_options(${target} PRIVATE /W4)
	else()
		target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
	endif()

	if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
		target_compile_options(${target} PRIVATE -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wimplicit-fallthrough)
	endif()

	if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
		target_compile_options(${target} PRIVATE -Wno-missing-designated-field-initializers)
	endif()
endfunction()

add_library(bedsim STATIC
	src/physics.cpp
	src/semantics.cpp
)
add_library(bedsim::bedsim ALIAS bedsim)
target_include_directories(bedsim PUBLIC include)
target_compile_features(bedsim PUBLIC cxx_std_23)
bedsim_enable_warnings(bedsim)

if(MSVC AND NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
	target_compile_options(bedsim PUBLIC /fp:precise)
else()
	target_compile_options(bedsim PUBLIC $<IF:$<BOOL:${MSVC}>,/clang:-ffp-contract=off,-ffp-contract=off>)
endif()

if(BEDSIM_BUILD_TESTS)
	enable_testing()
	add_subdirectory(tests)
endif()
