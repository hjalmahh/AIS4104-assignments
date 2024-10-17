find_package(Eigen3 CONFIG REQUIRED)

add_executable(assigment-3 main.cpp)

target_link_libraries(assigment-3 PUBLIC math PRIVATE Eigen3::Eigen)
