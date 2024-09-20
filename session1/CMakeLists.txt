find_package(Eigen3 CONFIG REQUIRED)

add_executable(session1 main.cpp)

target_link_libraries(session1 PUBLIC math PRIVATE Eigen3::Eigen)
