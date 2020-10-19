project(divergence_meter_project_v2 NONE)
cmake_minimum_required(VERSION 3.15)

file(GLOB_RECURSE SRC "src/*.rs" ".cargo/*")

string(TOLOWER ${CMAKE_BUILD_TYPE} _folder)
set(ELF ${PROJECT_SOURCE_DIR}/target/thumbv7em-none-eabihf/${_folder}/${PROJECT_NAME})

if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_custom_target(Elf
            COMMAND cargo build
            DEPENDS ${SRC}
            BYPRODUCTS ${ELF}
            )
elseif (CMAKE_BUILD_TYPE STREQUAL "Release")
    add_custom_target(Elf
            COMMAND cargo build --release
            DEPENDS ${SRC}
            BYPRODUCTS ${ELF}
            )
else ()
    message(FATAL_ERROR "Unsupported build type: ${CMAKE_BUILD_TYPE}")
endif ()


add_custom_target(Bin
        COMMAND arm-none-eabi-objcopy -O binary ${ELF} ${ELF}.bin
        DEPENDS ${ELF}
        BYPRODUCTS ${ELF}.bin
        )

add_custom_target(
        Flash ALL
        COMMAND arm-none-eabi-gdb -q -ex "set confirm off" -ex "target extended-remote :3333" -ex "load" -ex "kill" -ex "quit" ${ELF}
        DEPENDS ${ELF}
)
