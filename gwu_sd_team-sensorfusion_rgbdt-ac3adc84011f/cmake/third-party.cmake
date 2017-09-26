
# Load 3rd party libraries
function (third_party)

	# Load OpenCV
	find_package(OpenCV REQUIRED)
	set(OpenCV_FOUND ${OpenCV_FOUND} CACHE INTERNAL "OpenCV: Library found" FORCE)		
	if(OpenCV_FOUND)
		set(PKG_LIBRARIES ${OpenCV_LIBRARIES} ${OpenCV_LIBRARY} ${OpenCV_LIBS})
		set(PKG_INCLUDES ${OpenCV_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIR})

		set(OpenCV_LIBRARIES  ${PKG_LIBRARIES}
			CACHE INTERNAL "OpenCV: Project Library" FORCE)
		set(OpenCV_INCLUDE_DIRS ${PKG_INCLUDES}
			CACHE INTERNAL "OpenCV: Include Directories" FORCE)
		message(STATUS "OpenCV found")
	else()
		message(STATUS "OpenCV not found")
	endif()

	# Load Boost
	set(Boost_USE_STATIC_LIBS ON)
	set(Boost_USE_MULTITHREADED ON)
	#set(Boost_DEBUG ON)
	find_package(Boost 1.60.0 REQUIRED COMPONENTS date_time thread)
	set(Boost_FOUND ${Boost_FOUND} CACHE INTERNAL "Boost: Library found" FORCE)		
	if(Boost_FOUND)
		set(PKG_LIBRARIES ${Boost_LIBRARIES} ${Boost_LIBRARY} ${Boost_LIBS})
		set(PKG_INCLUDES ${Boost_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})

		set(Boost_LIBRARIES  ${PKG_LIBRARIES}
			CACHE INTERNAL "Boost: Project Library" FORCE)
		set(Boost_INCLUDE_DIRS ${PKG_INCLUDES}
			CACHE INTERNAL "Boost: Include Directories" FORCE)
		message(STATUS "Boost found")
	else()
		message(STATUS "Boost not found")
	endif()

	# KinectSDK2
	find_package( KinectSDK2 REQUIRED )
	if (KinectSDK2_FOUND)
		set(PKG_LIBRARIES ${KinectSDK2_LIBRARIES} ${KinectSDK2_LIBRARY} ${KinectSDK2_LIBS})
		set(PKG_INCLUDES ${KinectSDK2_INCLUDE_DIRS} ${KinectSDK2_INCLUDE_DIR})

		set(KinectSDK2_LIBRARIES  ${PKG_LIBRARIES}
			CACHE INTERNAL "OpenCV: Project Library" FORCE)
		set(KinectSDK2_INCLUDE_DIRS ${PKG_INCLUDES}
			CACHE INTERNAL "OpenCV: Include Directories" FORCE)
		message(STATUS "KinectSDK2 found")
	else()
		message(STATUS "KinectSDK2 not found")
	endif()

	# ZMQ
	find_package(ZMQ REQUIRED)
	if (ZMQ_FOUND)
		set(PKG_LIBRARIES ${ZMQ_LIBRARIES} ${ZMQ_LIBRARY} ${ZMQ_LIBS})
		set(PKG_INCLUDES ${ZMQ_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIR})

		set(ZMQ_LIBRARIES  ${PKG_LIBRARIES}
			CACHE INTERNAL "ZMQ: Project Library" FORCE)
		set(ZMQ_INCLUDE_DIRS ${PKG_INCLUDES}
			CACHE INTERNAL "ZMQ: Include Directories" FORCE)
		message(STATUS "ZMQ found")
	else()
		message(STATUS "ZMQ not found")
	endif()

endfunction(third_party)