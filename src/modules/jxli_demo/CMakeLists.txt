px4_add_module(
	MODULE modules__jxli_demo
	MAIN jxli_demo
	COMPILE_FLAGS
	SRCS
		jxliDemo.cpp
		jxliDemo.hpp
	DEPENDS
		controllib
		geo
		SlewRate
	)

