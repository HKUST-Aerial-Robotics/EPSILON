# TRY TO FIND THE INCLUDE DIRECTORY
find_path(NLOPT_INCLUDE_DIR
nlopt.h
HINTS ${PROJECT_SOURCE_DIR}/thirdparty/nlopt/include
)

if(NLOPT_INCLUDE_DIR)
  set(NLOPT_FOUND_INCLUDE TRUE)
  set(NLOPT_INCLUDE_DIRS
  ${NLOPT_INCLUDE_DIR})
  message(STATUS "Found NLOPT include dirs: ${NLOPT_INCLUDE_DIRS}")
else()
  message(STATUS "Could not find NLOPT include dir")
endif()

# TRY TO FIND THE LIBRARIES
set(NLOPT_LIBS_LIST
  nlopt
)

set(NLOPT_LIBRARIES)
set(NLOPT_FOUND_LIBS TRUE)
foreach(LIB ${NLOPT_LIBS_LIST})
  find_library(NLOPT_LIB_${LIB}
    NAMES ${LIB}
    HINTS ${PROJECT_SOURCE_DIR}/thirdparty/nlopt/lib)
  if(NLOPT_LIB_${LIB})
    set(NLOPT_LIBRARIES ${NLOPT_LIBRARIES} ${NLOPT_LIB_${LIB}})
  else()
    set(NLOPT_FOUND_LIBS FALSE)
  endif()
endforeach()

if(NLOPT_FOUND_LIBS)
  message(STATUS "Found NLOPT libraries: ${NLOPT_LIBRARIES}")
elseif()
  message(STATUS "Cound not find NLOPT libraries")
endif()

# SUCCESS if BOTH THE LIBRARIES AND THE INCLUDE DIRECTORIES WERE FOUND
if(NLOPT_FOUND_INCLUDE AND NLOPT_FOUND_LIBS)
  set(NLOPT_FOUND TRUE)
  message(STATUS "Found NLOPT")
elseif()
  message(STATUS "Cound not find NLOPT")
endif()