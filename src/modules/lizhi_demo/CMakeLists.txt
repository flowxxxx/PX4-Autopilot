px4_add_module(
	MODULE modules__lizhi_demo
	MAIN lizhi_demo
	COMPILE_FLAGS
	SRCS
		LiZhiDemo.cpp
		LiZhiDemo.hpp
	DEPENDS
		controllib
		geo
		SlewRate
	)
