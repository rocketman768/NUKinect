FIND_PROGRAM( DOXYGEN_CMD doxygen )

# Can type "make doc" to generate documentation.
IF( DOXYGEN_CMD )
  ADD_CUSTOM_TARGET( doc
                     COMMAND ${DOXYGEN_CMD} Doxyfile
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                   )
ELSE()
  ADD_CUSTOM_TARGET( doc
                     COMMAND "echo" "You do not have Doxygen."
                   )
ENDIF()
