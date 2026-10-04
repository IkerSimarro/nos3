project(hil_bridge C)

find_package(NOSENGINE REQUIRED QUIET COMPONENTS common transport client uart i2c spi can)

include_directories(../protocol
                    ${NOSENGINE_INCLUDE_DIRS})

add_executable(nos3-hil-bridge src/hil_bridge.c ../protocol/hil_link.c)
set_target_properties(nos3-hil-bridge PROPERTIES C_STANDARD 99 COMPILE_FLAGS "-Wall -Wextra")
find_package(Threads REQUIRED)
target_link_libraries(nos3-hil-bridge ${NOSENGINE_LIBRARIES} Threads::Threads)
install(TARGETS nos3-hil-bridge RUNTIME DESTINATION bin)
