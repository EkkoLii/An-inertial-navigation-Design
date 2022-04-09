# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Desktop/INS_Project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Desktop/INS_Project

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components

.PHONY : list_install_components/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Desktop/INS_Project/CMakeFiles /home/pi/Desktop/INS_Project//CMakeFiles/progress.marks
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/pi/Desktop/INS_Project/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named lsm9ds1_static

# Build rule for target.
lsm9ds1_static: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 lsm9ds1_static
.PHONY : lsm9ds1_static

# fast build rule for target.
lsm9ds1_static/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1_static.dir/build.make CMakeFiles/lsm9ds1_static.dir/build
.PHONY : lsm9ds1_static/fast

#=============================================================================
# Target rules for targets named lsm9ds1

# Build rule for target.
lsm9ds1: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 lsm9ds1
.PHONY : lsm9ds1

# fast build rule for target.
lsm9ds1/fast:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1.dir/build.make CMakeFiles/lsm9ds1.dir/build
.PHONY : lsm9ds1/fast

#=============================================================================
# Target rules for targets named LSM9DS1_demo

# Build rule for target.
LSM9DS1_demo: cmake_check_build_system
	$(MAKE) $(MAKESILENT) -f CMakeFiles/Makefile2 LSM9DS1_demo
.PHONY : LSM9DS1_demo

# fast build rule for target.
LSM9DS1_demo/fast:
	$(MAKE) $(MAKESILENT) -f example/CMakeFiles/LSM9DS1_demo.dir/build.make example/CMakeFiles/LSM9DS1_demo.dir/build
.PHONY : LSM9DS1_demo/fast

LSM9DS1.o: LSM9DS1.cpp.o

.PHONY : LSM9DS1.o

# target to build an object file
LSM9DS1.cpp.o:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1_static.dir/build.make CMakeFiles/lsm9ds1_static.dir/LSM9DS1.cpp.o
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1.dir/build.make CMakeFiles/lsm9ds1.dir/LSM9DS1.cpp.o
.PHONY : LSM9DS1.cpp.o

LSM9DS1.i: LSM9DS1.cpp.i

.PHONY : LSM9DS1.i

# target to preprocess a source file
LSM9DS1.cpp.i:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1_static.dir/build.make CMakeFiles/lsm9ds1_static.dir/LSM9DS1.cpp.i
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1.dir/build.make CMakeFiles/lsm9ds1.dir/LSM9DS1.cpp.i
.PHONY : LSM9DS1.cpp.i

LSM9DS1.s: LSM9DS1.cpp.s

.PHONY : LSM9DS1.s

# target to generate assembly for a file
LSM9DS1.cpp.s:
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1_static.dir/build.make CMakeFiles/lsm9ds1_static.dir/LSM9DS1.cpp.s
	$(MAKE) $(MAKESILENT) -f CMakeFiles/lsm9ds1.dir/build.make CMakeFiles/lsm9ds1.dir/LSM9DS1.cpp.s
.PHONY : LSM9DS1.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... install"
	@echo "... install/local"
	@echo "... install/strip"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... LSM9DS1_demo"
	@echo "... lsm9ds1"
	@echo "... lsm9ds1_static"
	@echo "... LSM9DS1.o"
	@echo "... LSM9DS1.i"
	@echo "... LSM9DS1.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system
