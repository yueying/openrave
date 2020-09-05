MACRO(REMOVE_MATCHING_FILES_FROM_LIST match_expr lst_files)
	SET(lst_files_aux "")
	FOREACH(FIL ${${lst_files}})
		IF(NOT ${FIL} MATCHES "${match_expr}")
			SET(lst_files_aux "${lst_files_aux}" "${FIL}")
		ENDIF(NOT ${FIL} MATCHES "${match_expr}")
	ENDFOREACH(FIL)
	SET(${lst_files} ${lst_files_aux})
ENDMACRO(REMOVE_MATCHING_FILES_FROM_LIST)

function(openravepy_add_example example_name)
  if(${ARGC} GREATER 1)
    set(source ${ARGN})
  else()
    set(source "${example_name}.py")
  endif()
  add_custom_target(${example_name}
    COMMAND ${CMAKE_COMMAND} -E echo "Running examples by: ${PYTHON_EXECUTABLE} ${source}"
    COMMAND ${PYTHON_EXECUTABLE} ${source}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    SOURCES ${source}
  )
endfunction()

FILE(GLOB listexamples 
"./*.py" 
)

REMOVE_MATCHING_FILES_FROM_LIST(".*init__.py" listexamples)

FOREACH(FILE_PATH ${listexamples})
    STRING(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FILE_PATH})
    openravepy_add_example(${FILE_NAME} "${FILE_PATH}")
	if(ENABLE_SOLUTION_FOLDERS)
	set_target_properties(${FILE_NAME} PROPERTIES FOLDER "python samples")
    endif(ENABLE_SOLUTION_FOLDERS)
ENDFOREACH(FILE_PATH)

