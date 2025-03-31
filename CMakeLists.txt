cmake_minimum_required(VERSION 3.14)

project(c_modbus_lib LANGUAGES C)

set(CMAKE_C_STANDARD 11) # Устанавливаем стандарт C11 (или другой, если нужно)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(c_modbus_lib
    modbus.c
    modbus.h
    main.c
)

include(GNUInstallDirs)
install(TARGETS c_modbus_lib
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
