cmake_minimum_required(VERSION 3.16)

find_package( OpenCV REQUIRED )

PROJECT(STag VERSION 0.0.0 LANGUAGES CXX)

FILE(GLOB all_source_files src/*.cpp)
FILE(GLOB ED_sources src/ED/*.cpp)

LIST(APPEND all_source_files ${ED_sources})

add_executable(main ${all_source_files})
target_link_libraries(main ${OpenCV_LIBS})
