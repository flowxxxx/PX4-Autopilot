px4_add_module(
	MODULE modules__pengbo_demo
	MAIN pengbo_demo
	COMPILE_FLAGS
	SRCS
		pengboDemo.cpp
		pengboDemo.hpp
	DEPENDS
		controllib
		geo
		SlewRate
	)
