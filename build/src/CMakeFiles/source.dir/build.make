# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/leotseng/.local/bin/cmake

# The command to remove a file.
RM = /home/leotseng/.local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/leotseng/test_github

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/leotseng/test_github/build

# Include any dependencies generated for this target.
include src/CMakeFiles/source.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include src/CMakeFiles/source.dir/compiler_depend.make

# Include the progress variables for this target.
include src/CMakeFiles/source.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/source.dir/flags.make

src/CMakeFiles/source.dir/Printer.cpp.o: src/CMakeFiles/source.dir/flags.make
src/CMakeFiles/source.dir/Printer.cpp.o: /home/leotseng/test_github/src/Printer.cpp
src/CMakeFiles/source.dir/Printer.cpp.o: src/CMakeFiles/source.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/leotseng/test_github/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/source.dir/Printer.cpp.o"
	cd /home/leotseng/test_github/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT src/CMakeFiles/source.dir/Printer.cpp.o -MF CMakeFiles/source.dir/Printer.cpp.o.d -o CMakeFiles/source.dir/Printer.cpp.o -c /home/leotseng/test_github/src/Printer.cpp

src/CMakeFiles/source.dir/Printer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/source.dir/Printer.cpp.i"
	cd /home/leotseng/test_github/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/leotseng/test_github/src/Printer.cpp > CMakeFiles/source.dir/Printer.cpp.i

src/CMakeFiles/source.dir/Printer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/source.dir/Printer.cpp.s"
	cd /home/leotseng/test_github/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/leotseng/test_github/src/Printer.cpp -o CMakeFiles/source.dir/Printer.cpp.s

# Object files for target source
source_OBJECTS = \
"CMakeFiles/source.dir/Printer.cpp.o"

# External object files for target source
source_EXTERNAL_OBJECTS =

src/libsource.a: src/CMakeFiles/source.dir/Printer.cpp.o
src/libsource.a: src/CMakeFiles/source.dir/build.make
src/libsource.a: src/CMakeFiles/source.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/leotseng/test_github/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libsource.a"
	cd /home/leotseng/test_github/build/src && $(CMAKE_COMMAND) -P CMakeFiles/source.dir/cmake_clean_target.cmake
	cd /home/leotseng/test_github/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/source.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/source.dir/build: src/libsource.a
.PHONY : src/CMakeFiles/source.dir/build

src/CMakeFiles/source.dir/clean:
	cd /home/leotseng/test_github/build/src && $(CMAKE_COMMAND) -P CMakeFiles/source.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/source.dir/clean

src/CMakeFiles/source.dir/depend:
	cd /home/leotseng/test_github/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/leotseng/test_github /home/leotseng/test_github/src /home/leotseng/test_github/build /home/leotseng/test_github/build/src /home/leotseng/test_github/build/src/CMakeFiles/source.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/source.dir/depend

