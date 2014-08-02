##############################################################################
# search paths
##############################################################################
SET(ART_INCLUDE_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/inc/art
  ${ART_INCLUDE_DIRS}
  ${ART_INCLUDE_SEARCH_DIR}
  "/home/steppo/Desktop/my-svn/multiViewTools/libART/include"
)

SET(ART_LIBRARY_SEARCH_DIRS
  ${GLOBAL_EXT_DIR}/lib
  ${ART_LIBRARY_DIRS}
  ${ART_LIBRARY_SEARCH_DIR}
  "/home/steppo/Desktop/my-svn/multiViewTools/libART/lib"
)

##############################################################################
# feedback to provide user-defined paths to search for art
##############################################################################
MACRO (request_art_search_directories)
    
  IF ( NOT ART_INCLUDE_DIRS AND NOT ART_LIBRARY_DIRS )
    SET(ART_INCLUDE_SEARCH_DIR "Please provide art include path." CACHE PATH "path to art headers.")
    SET(ART_LIBRARY_SEARCH_DIR "Please provide art library path." CACHE PATH "path to art libraries.")
    MESSAGE(FATAL_ERROR "find_art.cmake: unable to find art.")
  ENDIF ( NOT ART_INCLUDE_DIRS AND NOT ART_LIBRARY_DIRS )

  IF ( NOT ART_INCLUDE_DIRS )
    SET(ART_INCLUDE_SEARCH_DIR "Please provide art include path." CACHE PATH "path to art headers.")
    MESSAGE(FATAL_ERROR "find_art.cmake: unable to find art headers.")
  ELSE ( NOT ART_INCLUDE_DIRS )
    UNSET(ART_INCLUDE_SEARCH_DIR CACHE)
  ENDIF ( NOT ART_INCLUDE_DIRS )

  IF ( NOT ART_LIBRARY_DIRS )
    SET(ART_LIBRARY_SEARCH_DIR "Please provide art library path." CACHE PATH "path to art libraries.")
    MESSAGE(FATAL_ERROR "find_art.cmake: unable to find art libraries.")
  ELSE ( NOT ART_LIBRARY_DIRS )
    UNSET(ART_LIBRARY_SEARCH_DIR CACHE)
  ENDIF ( NOT ART_LIBRARY_DIRS ) 

ENDMACRO (request_art_search_directories)

##############################################################################
# search
##############################################################################
message(STATUS "-- checking for ART")

IF (NOT ART_INCLUDE_DIRS)

  SET(_ART_FOUND_INC_DIRS "")
  FOREACH(_SEARCH_DIR ${ART_INCLUDE_SEARCH_DIRS})
    FIND_PATH(_CUR_SEARCH
      NAMES ARTListener.h
        PATHS ${_SEARCH_DIR}
        NO_DEFAULT_PATH)
    IF (_CUR_SEARCH)
      LIST(APPEND _ART_FOUND_INC_DIRS ${_CUR_SEARCH})
    ENDIF(_CUR_SEARCH)
    SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
  ENDFOREACH(_SEARCH_DIR ${ART_INCLUDE_SEARCH_DIRS})

  IF (NOT _ART_FOUND_INC_DIRS)
    request_art_search_directories()
  ENDIF (NOT _ART_FOUND_INC_DIRS)

  FOREACH(_INC_DIR ${_ART_FOUND_INC_DIRS})
    SET(ART_INCLUDE_DIRS ${ART_INCLUDE_DIRS} ${_INC_DIR} CACHE PATH "art include directory.")
  ENDFOREACH(_INC_DIR ${_ART_FOUND_INC_DIRS})

ENDIF (NOT ART_INCLUDE_DIRS)

IF(UNIX)
  SET(ART_LIB_FILENAME "libART.so")
ELSEIF(WIN32)
  SET(ART_LIB_FILENAME "libART.lib")
ENDIF(UNIX)

IF ( NOT ART_LIBRARY_DIRS )

  SET(_ART_FOUND_LIB_DIR "")
  SET(_ART_POSTFIX "")

  FOREACH(_SEARCH_DIR ${ART_LIBRARY_SEARCH_DIRS})
    FIND_PATH(_CUR_SEARCH
      NAMES ${ART_LIB_FILENAME}
        PATHS ${_SEARCH_DIR}
        PATH_SUFFIXES release debug
        NO_DEFAULT_PATH)
    IF (_CUR_SEARCH)
      LIST(APPEND _ART_FOUND_LIB_DIR ${_SEARCH_DIR})
    ENDIF(_CUR_SEARCH)
    SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
  ENDFOREACH(_SEARCH_DIR ${ART_LIBRARY_SEARCH_DIRS})

  IF (NOT _ART_FOUND_LIB_DIR)
    request_art_search_directories()
  ELSE (NOT _ART_FOUND_LIB_DIR)
    SET(ART_LIBRARY_DIRS ${_ART_FOUND_LIB_DIR} CACHE PATH "The art library directory")
  ENDIF (NOT _ART_FOUND_LIB_DIR)

  FOREACH(_LIB_DIR ${_ART_FOUND_LIB_DIR})
    LIST(APPEND _ART_LIBRARIES ${ART_LIB_FILENAME})
  ENDFOREACH(_LIB_DIR ${_ART_FOUND_INC_DIRS})

  IF (_ART_FOUND_LIB_DIR)
    SET(ART_LIBRARIES ${_ART_LIBRARIES} CACHE FILEPATH "The art library filename.")
  ENDIF (_ART_FOUND_LIB_DIR)

ENDIF ( NOT ART_LIBRARY_DIRS )

##############################################################################
# verify
##############################################################################
IF ( NOT ART_INCLUDE_DIRS OR NOT ART_LIBRARY_DIRS )
  request_art_search_directories()
ELSE ( NOT ART_INCLUDE_DIRS OR NOT ART_LIBRARY_DIRS ) 
  UNSET(ART_INCLUDE_SEARCH_DIR CACHE)
  UNSET(ART_LIBRARY_SEARCH_DIR CACHE)
  MESSAGE(STATUS "--  found matching art version")
ENDIF ( NOT ART_INCLUDE_DIRS OR NOT ART_LIBRARY_DIRS )
