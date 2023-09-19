# Finds if THUNDERSVM is installed and determines the locations of the
# include headers and library files.
#
# Written by Sihui Li.

include(FindPackageHandleStandardArgs)

find_library(THUNDERSVM_LIBRARY thundersvm DOC "Location of ThunderSVM library")
if(THUNDERSVM_LIBRARY AND NOT THUNDERSVM_INCLUDE_DIRS)
    set(THUNDERSVM_INCLUDE_DIRS "/usr/local/include")
endif()
find_path(THUNDERSVM_INCLUDE_DIR thundersvm.h PATH_SUFFIXES thundersvm
    DOC "Location of ThunderSVM header file directory")
find_package_handle_standard_args(thundersvm DEFAULT_MSG THUNDERSVM_LIBRARY THUNDERSVM_INCLUDE_DIR)
mark_as_advanced(THUNDERSVM_LIBRARY THUNDERSVM_INCLUDE_DIR)
