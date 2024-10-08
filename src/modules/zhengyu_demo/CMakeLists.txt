px4_add_module(
    MODULE modules__zhengyu_demo
    MAIN zhengyu_demo
    COMPILE_FLAGS
    SRCS
        ZhengYuDemo.cpp
        ZhengYuDemo.hpp
    DEPENDS
        controllib
        geo
        SlewRate
)