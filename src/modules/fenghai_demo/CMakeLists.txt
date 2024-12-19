px4_add_module(
	MODULE modules__fenghai_demo
	MAIN fenghai_demo
	COMPILE_FLAGS
	SRCS
		FengHaiDemo.cpp
		FengHaiDemo.hpp
	DEPENDS
		controllib
		geo
		SlewRate
)
