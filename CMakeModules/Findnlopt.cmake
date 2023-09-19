# Find NLOPT, a nonlinear optimization solver
# Written by Sihui Li.

include(FindPackageHandleStandardArgs)

find_package(PkgConfig)
if(PKGCONFIG_FOUND)
    pkg_check_modules(NLOPT nlopt)
    if(NLOPT_LIBRARIES AND NOT NLOPT_INCLUDE_DIRS)
        set(NLOPT_INCLUDE_DIRS "/usr/include")
    endif()

    if (NLOPT_LIBRARY_DIRS)
      foreach(FDIR ${NLOPT_LIBRARY_DIRS})
        string(REGEX MATCH "^-l" FOUND ${FDIR})
        if (FOUND)
          string(REGEX REPLACE "^-l" "" FDIR2 ${FDIR})
          list(PREPEND NLOPT_LIBRARIES ${FDIR2})
        else()
          list(APPEND NLOPT_LIBRARY_DIRS_TEMP ${FDIR})
        endif()
      endforeach()

      set(NLOPT_LIBRARY_DIRS ${NLOPT_LIBRARY_DIRS_TEMP})
    endif()
endif()

find_package_handle_standard_args(nlopt DEFAULT_MSG NLOPT_LIBRARY_DIRS NLOPT_LIBRARIES NLOPT_INCLUDE_DIRS)
