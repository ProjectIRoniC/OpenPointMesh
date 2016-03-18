# The purpose of this Find<package>.cmake is to find and use pkg-config (.pc) files.
# This is used when the package does not have a CMakeLists.txt.
# By using the pkg-config files we can take advantage of IMPORT_TARGETS.

PKG_CHECK_MODULES( FontConfig REQUIRED )
