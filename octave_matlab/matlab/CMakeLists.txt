#
# generate mex files
#
message(STATUS "build with matlab ${OCTAVEMATLAB_FILES_DIR}")
set(MATLAB_MEX_OUT ${CMAKE_CURRENT_BINARY_DIR})

matlab_add_mex(NAME orcreate SRC ${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp ${OCTAVEMATLAB_FILES_DIR}/socketconnect.h)
matlab_add_mex(NAME orread SRC ${OCTAVEMATLAB_FILES_DIR}/orread.cpp ${OCTAVEMATLAB_FILES_DIR}/socketconnect.h)
matlab_add_mex(NAME orwrite SRC ${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp ${OCTAVEMATLAB_FILES_DIR}/socketconnect.h)
add_dependencies(orread orcreate) # used to force mex to be called once at a time
add_dependencies(orwrite orread) # used to force mex to be called once at a time

if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(orcreate PROPERTIES FOLDER "matlab")
	set_target_properties(orread PROPERTIES FOLDER "matlab")
	set_target_properties(orwrite PROPERTIES FOLDER "matlab")
endif(ENABLE_SOLUTION_FOLDERS)

# because MATLAB is non-free, have to distribute the source files
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}MATLAB_DEPENDS ${COMPONENT_PREFIX}base PARENT_SCOPE)
set(CPACK_COMPONENT_${COMPONENT_PREFIX_UPPER}MATLAB_DISPLAY_NAME "Matlab Bindings" PARENT_SCOPE)
set(CPACK_COMPONENTS_ALL ${CPACK_COMPONENTS_ALL} ${COMPONENT_PREFIX}matlab PARENT_SCOPE)

install(FILES "${OCTAVEMATLAB_FILES_DIR}/orcreate.cpp" "${OCTAVEMATLAB_FILES_DIR}/socketconnect.h" "${OCTAVEMATLAB_FILES_DIR}/orread.cpp" "${OCTAVEMATLAB_FILES_DIR}/orwrite.cpp" DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
if( WIN32 OR WIN64 )
  install(FILES runmex.bat DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
endif()

install(TARGETS orcreate DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
install(TARGETS orread DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
install(TARGETS orwrite DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
install(FILES ${OCTAVEMATLAB_FILES} DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab)
install(DIRECTORY "${OCTAVEMATLAB_FILES_DIR}/examples" DESTINATION ${OPENRAVE_MATLAB_INSTALL_DIR} COMPONENT ${COMPONENT_PREFIX}matlab PATTERN ".svn" EXCLUDE)
