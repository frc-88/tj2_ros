
find_library(LibDS_LIBRARIES
    NAMES
    libds
    LibDS
    PATH_SUFFIXES
    lib
)

find_path(LibDS_INCLUDE_DIRS
    NAMES
    libds/LibDS.h
    PATH_SUFFIXES
    include
)

message(STATUS LibDS_LIBRARIES: ${LibDS_LIBRARIES})
message(STATUS LibDS_INCLUDE_DIRS: ${LibDS_INCLUDE_DIRS})
