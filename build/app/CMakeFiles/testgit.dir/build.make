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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.25.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.25.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/tsengweizhe/Desktop/Test_Github/Test_Github

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build

# Include any dependencies generated for this target.
include app/CMakeFiles/testgit.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include app/CMakeFiles/testgit.dir/compiler_depend.make

# Include the progress variables for this target.
include app/CMakeFiles/testgit.dir/progress.make

# Include the compile flags for this target's objects.
include app/CMakeFiles/testgit.dir/flags.make

app/CMakeFiles/testgit.dir/main.cpp.o: app/CMakeFiles/testgit.dir/flags.make
app/CMakeFiles/testgit.dir/main.cpp.o: /Users/tsengweizhe/Desktop/Test_Github/Test_Github/app/main.cpp
app/CMakeFiles/testgit.dir/main.cpp.o: app/CMakeFiles/testgit.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object app/CMakeFiles/testgit.dir/main.cpp.o"
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT app/CMakeFiles/testgit.dir/main.cpp.o -MF CMakeFiles/testgit.dir/main.cpp.o.d -o CMakeFiles/testgit.dir/main.cpp.o -c /Users/tsengweizhe/Desktop/Test_Github/Test_Github/app/main.cpp

app/CMakeFiles/testgit.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testgit.dir/main.cpp.i"
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/tsengweizhe/Desktop/Test_Github/Test_Github/app/main.cpp > CMakeFiles/testgit.dir/main.cpp.i

app/CMakeFiles/testgit.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testgit.dir/main.cpp.s"
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app && /Library/Developer/CommandLineTools/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/tsengweizhe/Desktop/Test_Github/Test_Github/app/main.cpp -o CMakeFiles/testgit.dir/main.cpp.s

# Object files for target testgit
testgit_OBJECTS = \
"CMakeFiles/testgit.dir/main.cpp.o"

# External object files for target testgit
testgit_EXTERNAL_OBJECTS =

app/testgit: app/CMakeFiles/testgit.dir/main.cpp.o
app/testgit: app/CMakeFiles/testgit.dir/build.make
app/testgit: src/libsource.a
app/testgit: app/CMakeFiles/testgit.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable testgit"
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testgit.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
app/CMakeFiles/testgit.dir/build: app/testgit
.PHONY : app/CMakeFiles/testgit.dir/build

app/CMakeFiles/testgit.dir/clean:
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app && $(CMAKE_COMMAND) -P CMakeFiles/testgit.dir/cmake_clean.cmake
.PHONY : app/CMakeFiles/testgit.dir/clean

app/CMakeFiles/testgit.dir/depend:
	cd /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/tsengweizhe/Desktop/Test_Github/Test_Github /Users/tsengweizhe/Desktop/Test_Github/Test_Github/app /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app /Users/tsengweizhe/Desktop/Test_Github/Test_Github/build/app/CMakeFiles/testgit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : app/CMakeFiles/testgit.dir/depend

