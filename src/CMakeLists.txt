add_library(cyclone 
			core.cpp
			particle.cpp
			pfgen.cpp
			pforces.cpp
			pcontacts.cpp)


target_include_directories(cyclone PUBLIC 
    "${CMAKE_SOURCE_DIR}/include"
)