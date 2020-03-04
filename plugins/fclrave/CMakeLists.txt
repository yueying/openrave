###########################################
# fclrave openrave plugin
###########################################

message(STATUS "Looking for fcl")
find_package(FCL)
message(STATUS "FCL_FOUND = ${FCL_FOUND}")

if( FCL_FOUND )
  #STRING( REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" FCL_MINOR_VERSION ${FCL_VERSION})

  if(FCL_MINOR_VERSION GREATER "4")
    set(CMAKE_REQUIRED_INCLUDES ${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
    set(CMAKE_REQUIRED_FLAGS "-std=c++11")
    foreach(FCL_LIBRARIES_DIR ${FCL_LIBRARIES_DIRS})
      set(CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS} -L${FCL_LIBRARIES_DIR}")
    endforeach()
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
    else()
      message(WARNING "FCL_HAS_REPLACEOBJECT = ${FCL_HAS_REPLACEOBJECT}")
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
    else()
      message(WARNING "FCL_SUPPORT_BULK_UPDATE = ${FCL_SUPPORT_BULK_UPDATE}")
    endif()

    link_directories(${OPENRAVE_LINK_DIRS} ${FCL_LIBRARY_DIRS})
    include_directories(${FCL_INCLUDE_DIRS} ${FCL_INCLUDEDIR})
    add_library(fclrave SHARED fclrave.cpp fclcollision.h fclstatistics.h fclspace.h plugindefs.h)
    target_link_libraries(fclrave libopenrave ${FCL_LIBRARIES})
    target_link_libraries(fclrave PRIVATE boost_assertion_failed)
    if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX OR COMPILER_IS_CLANG)
      add_definitions("-std=c++11")
    endif()
    set_target_properties(fclrave PROPERTIES COMPILE_FLAGS "${PLUGIN_COMPILE_FLAGS} ${FCL_CFLAGS_OTHER} -std=c++11" LINK_FLAGS "${PLUGIN_LINK_FLAGS} ${FCL_LDFLAGS}")
    install(TARGETS fclrave DESTINATION ${OPENRAVE_PLUGINS_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}plugin-fclrave)
  else()
    message(STATUS "Could not find FCL. Please install FCL (https://github.com/flexible-collision-library/fcl)")
  endif()

  set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}PLUGIN-FCLRAVE_DISPLAY_NAME "Plugin for Flexible Collision Library (fcl) Collision" PARENT_SCOPE)
  set(PLUGIN_COMPONENT ${COMPONENT_PREFIX}plugin-fclrave PARENT_SCOPE)
endif()

# restore the link dirs
link_directories(${OPENRAVE_LINK_DIRS})
