
px4_add_module(
	MODULE modules__nova_intercept
	MAIN nova_intercept
	SRCS
		nova_intercept.cpp
        	nova_intercept.hpp
	DEPENDS
		target_uav_predictor
		intercept_guidance
		geo
	)
