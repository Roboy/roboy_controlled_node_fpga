include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ncurses_arm/include)
set(ncurses_LIBRARY ${CMAKE_CURRENT_SOURCE_DIR}/ncurses_arm/lib/libncurses.so
        )
message(STATUS "ncurses lib: ${ncurses_LIBRARY}" )