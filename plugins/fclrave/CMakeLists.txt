###########################################
# fclrave openrave plugin
###########################################
find_package(FCL QUIET)
find_package(Eigen3 QUIET)
find_package(OCTOMAP QUIET)
find_package(ccd QUIET)
add_definitions(-D_ENABLE_EXTENDED_ALIGNED_STORAGE )
# If fcl-config.cmake is not found, use pkg-config and/or find_path() and
# find_library()
if(NOT fcl_FOUND)
  if(PKG_CONFIG_FOUND)
    pkg_check_modules(PC_FCL fcl)
    pkg_check_modules(PC_LIBFCL libfcl)
  endif()

  find_path(FCL_INCLUDE_DIR fcl/fcl.h
    HINTS "${PC_FCL_INCLUDE_DIRS}" "${PC_LIBFCL_INCLUDE_DIRS}")

  # Using find_library() even if pkg-config is available ensures that the full
  # path to the fcl library is available in FCL_LIBRARIES
  find_library(FCL_LIBRARY fcl
    HINTS "${PC_FCL_LIBRARY_DIRS}" "${PC_LIBFCL_LIBRARY_DIRS}")

  # libfcl links to LibM on UNIX.
  if(CYGWIN OR NOT WIN32)
    find_library(M_LIBRARY m)
  endif()

  if(FCL_INCLUDE_DIR AND FCL_LIBRARY)
    set(FCL_INCLUDE_DIRS "${FCL_INCLUDE_DIR}")
    set(FCL_LIBRARIES "${FCL_LIBRARY}" "${M_LIBRARY}")
    set(fcl_FOUND ON)

    mark_as_advanced(FCL_INCLUDE_DIR FCL_LIBRARY)
  endif()
endif()

INCLUDE_DIRECTORIES(  ${FCL_INCLUDE_DIR})

if(NARROW_COLLISION_CACHING)
  add_definitions(-DNARROW_COLLISION_CACHING)
endif()


if(FCL_USE_STATISTICS)
  add_definitions(-DFCLUSESTATISTICS)
  if(FCL_STATISTICS_DISPLAY_CONTINUOUSLY)
    add_definitions(-DFCL_STATISTICS_DISPLAY_CONTINUOUSLY)
  endif()
endif()

if(FCLRAVE_USE_COLLISION_STATISTICS)
  add_definitions(-DFCLRAVE_COLLISION_OBJECTS_STATISTICS)
endif()

if(FCLRAVE_DEBUG_COLLISION_OBJECTS)
  add_definitions(-DFCLRAVE_DEBUG_COLLISION_OBJECTS)
endif()

find_path(CCD_INCLUDE_DIR ccd/ccd.h)
find_library(CCD_LIBRARY ccd)

add_definitions(-DNOMINMAX)
message(STATUS "---------------before find FCL")
if( FCL_FOUND )
	message(STATUS "find FCL")
  #STRING( REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" FCL_MINOR_VERSION ${FCL_VERSION})

  #if(FCL_MINOR_VERSION GREATER "4")
    set(CMAKE_REQUIRED_INCLUDES ${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
    set(CMAKE_REQUIRED_FLAGS "${PLUGIN_COMPILE_FLAGS} ${FCL_CFLAGS_OTHER} -std=c++11 ${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
    set(CMAKE_REQUIRED_LIBRARIES fcl)
    check_cxx_source_compiles("
      #include <fcl/broadphase/broadphase.h>

      int main() {
        fcl::NaiveCollisionManager manager;
        manager.replaceObject(nullptr, nullptr, false);
        return 0;
      }"
      FCL_HAS_REPLACEOBJECT)

    if( FCL_HAS_REPLACEOBJECT )
      add_definitions(-DFCLRAVE_USE_REPLACEOBJECT)
    endif()


    check_cxx_source_compiles("
      #include <fcl/broadphase/broadphase.h>

      int main() {
        fcl::DynamicAABBTreeCollisionManager manager;
        std::vector<fcl::CollisionObject*> v;
        manager.update(v, false);
        return 0;
      }"
      FCL_SUPPORT_BULK_UPDATE)

    if( FCL_SUPPORT_BULK_UPDATE )
      add_definitions(-DFCLRAVE_USE_BULK_UPDATE)
    endif()

    link_directories(${OPENRAVE_LINK_DIRS} ${FCL_LIBRARY_DIRS} )
    include_directories(${FCL_INCLUDE_DIRS} ${FCL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ${CCD_INCLUDE_DIR} ${OCTOMAP_INCLUDE_DIRS})
    add_library(fclrave SHARED fclrave.cpp fclcollision.h fclstatistics.h fclspace.h plugindefs.h fclmanagercache.h)
    target_link_libraries(fclrave libopenrave ${Log4cxx_LIBRARIES}  ${FCL_LIBRARY} ${CCD_LIBRARY} ${OCTOMAP_LIBRARIES})
    target_link_libraries(fclrave PRIVATE boost_assertion_failed)
    if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
      add_definitions("-std=c++11")
    endif()
    set_target_properties(fclrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${FCL_CFLAGS_OTHER} -std=c++11" LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
    install(TARGETS fclrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-fclrave)
  #else()
  #  message(STATUS "Could not find FCL. Please install FCL (https://github.com/flexible-collision-library/fcl)")
  #endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FCLRAVE_DISPLAY_NAME "Plugin for Flexible Collision Library (fcl) Collision" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-fclrave PARENT_SCOPE)
endif()

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
