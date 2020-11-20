CMAKE_MINIMUM_REQUIRED(VERSION 2.8.9)

project(ExampleProjects)

#SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -Wall -Wextra -Wno-unused-function -Wno-unused-parameter -Wno-missing-field-initializers")

add_subdirectory(Ascii)
add_subdirectory(Bootloader)
add_subdirectory(Communications)
add_subdirectory(Logger)
add_subdirectory(LogReader)
