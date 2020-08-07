############################
# qtcoinrave openrave plugin
############################
find_package(Coin NAMES Inventor Coin REQUIRED)
find_package(SoQt REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system)
if (NOT SOQT_LIBRARY_FOUND)
  message(STATUS "WARNING: SoQt not found, disabling QtCoin GUI plugin. Please install SoQt (http://www.coin3d.org)")
endif ()

find_package(OpenGL)
if(NOT OPENGL_FOUND)
  message(STATUS "WARNING: OpenGL not found. This library is not necessary, but it is good to link with")
endif()

FIND_PACKAGE(Qt5 QUIET COMPONENTS Core Xml OpenGL Gui Widgets Network PrintSupport SerialPort SQL)
IF( NOT Qt5_FOUND)
	MESSAGE("Qt{4,5} not found. Install it and set Qt{4,5}_DIR accordingly")
	IF (WIN32)
		MESSAGE("  In Windows, Qt5_DIR should be something like C:/Qt/5.4/msvc2013_64_opengl/lib/cmake/Qt5")
	ENDIF()
ENDIF()

if(Qt5_FOUND)
  INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS} ${Qt5Network_INCLUDE_DIRS} ${Qt5PrintSupport_INCLUDE_DIRS} ${Qt5SerialPort_INCLUDE_DIRS} ${Qt5Sql_INCLUDE_DIRS})
  SET(MY_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES} ${Qt5Network_LIBRARIES} ${Qt5PrintSupport_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${Qt5Sql_LIBRARIES})
endif()

set(CMAKE_AUTOMOC ON)
ADD_DEFINITIONS(-DQT_DISABLE_DEPRECATED_BEFORE=0)

if (Qt5_FOUND)
  message(STATUS "Detected SoQt/Coin3D GUI, making plugin ${Coin_INCLUDE_DIR}----- ${SoQt_INCLUDE_DIRS}")
  include_directories(${OPENGL_INCLUDE_DIR} ${Coin_INCLUDE_DIR} ${SoQt_INCLUDE_DIRS} ${SoQt_INCLUDE_DIR} )
  link_directories(${OPENRAVE_LINK_DIRS}  ${Coin_LIB_DIR} ${SoQt_LIBRARY_DIRS})

  # generate rules for building source files from the resources
  set(QTCOIN_MOCS qtcoinviewer.h)
  #include(${QT_USE_FILE})
  #qt4_wrap_cpp(MOC_OUTPUT_FILES ${QTCOIN_MOCS})

  check_include_file(X11/Xlib.h HAVE_X11_XLIB_H)

   # build sources, moc'd sources, and rcc'd sources
  add_library(qtcoinrave SHARED ivselector.cpp item.cpp qtcoinviewer.cpp qtcoinrave.cpp ${MOC_OUTPUT_FILES} item.h ivselector.h qtcoin.h qtcoinviewer.h qtcameraviewer.h ivmodelloader.cpp)

  set(QTCOIN_COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS}")
  if( HAVE_X11_XLIB_H )
    set(QTCOIN_COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} -DHAVE_X11_XLIB_H")
  endif()
  set(QTCOIN_LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${COIN_LINK_FLAGS} ${SOQT_LINK_FLAGS}")

  set (extLIBS ${MY_QT_LIBRARIES} ${LIBXML2_LIBRARIES} ${Boost_LIBRARIES} ${Coin_LIBRARIES} ${SoQt_LIBRARIES}  ${OPENGL_LIBRARIES})
  if( MSVC )
    set(extLIBS ${extLIBS} winmm imm32 ws2_32)
  endif()

  target_link_libraries(qtcoinrave ${extLIBS} libopenrave)
  target_link_libraries(qtcoinrave PRIVATE boost_assertion_failed)
  set_target_properties(qtcoinrave PROPERTIES COMPILE_FLAGS "${QTCOIN_COMPILE_FLAGS} ${SOQT_CXXFLAGS} ${COIN_CXXFLAGS}" LINK_FLAGS "${QTCOIN_LINK_FLAGS}")
  install(TARGETS qtcoinrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave)
  
  if( MSVC )
    # have to install the DLLs
    install(DIRECTORY "${SOQT_LINK_DIRS}/" DESTINATION bin COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave FILES_MATCHING PATTERN "*-${MSVC_PREFIX}-*.dll")
  endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-QTCOINRAVE_DISPLAY_NAME "Plugin for a Qt/Coin3d/SoQt viewer" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-qtcoinrave PARENT_SCOPE)
  
else()
  message(STATUS "WARNING: Could not find Coin3D/SoQt/Qt libraries, will not compile qtcoinrave viewer.")
endif()
