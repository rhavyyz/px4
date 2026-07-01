px4_add_module(
	MODULE modules__safe_land_provider
	MAIN safe_land_provider
	SRCS
		SafeLandProvider.cpp
		SafeLandProvider.hpp
	DEPENDS
		px4_work_queue
		geo
	)
