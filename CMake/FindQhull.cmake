# The purpose of this Find<package>.cmake is to force use of find_package() config mode.
# By using the CMake package config files we can take advantage of IMPORT_TARGETS.

FIND_PACKAGE( Qhull REQUIRED CONFIG )
