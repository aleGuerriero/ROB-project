# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
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
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ricca/catkin_ws/src/ROB-project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ricca/catkin_ws/src/ROB-project/build

# Utility rule file for project_generate_messages_py.

# Include the progress variables for this target.
include CMakeFiles/project_generate_messages_py.dir/progress.make

CMakeFiles/project_generate_messages_py: devel/lib/python3/dist-packages/project/msg/_Error_msg.py
CMakeFiles/project_generate_messages_py: devel/lib/python3/dist-packages/project/msg/__init__.py


devel/lib/python3/dist-packages/project/msg/_Error_msg.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
devel/lib/python3/dist-packages/project/msg/_Error_msg.py: ../msg/Error_msg.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ricca/catkin_ws/src/ROB-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Python from MSG project/Error_msg"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/ricca/catkin_ws/src/ROB-project/msg/Error_msg.msg -Iproject:/home/ricca/catkin_ws/src/ROB-project/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p project -o /home/ricca/catkin_ws/src/ROB-project/build/devel/lib/python3/dist-packages/project/msg

devel/lib/python3/dist-packages/project/msg/__init__.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
devel/lib/python3/dist-packages/project/msg/__init__.py: devel/lib/python3/dist-packages/project/msg/_Error_msg.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ricca/catkin_ws/src/ROB-project/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Python msg __init__.py for project"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/ricca/catkin_ws/src/ROB-project/build/devel/lib/python3/dist-packages/project/msg --initpy

project_generate_messages_py: CMakeFiles/project_generate_messages_py
project_generate_messages_py: devel/lib/python3/dist-packages/project/msg/_Error_msg.py
project_generate_messages_py: devel/lib/python3/dist-packages/project/msg/__init__.py
project_generate_messages_py: CMakeFiles/project_generate_messages_py.dir/build.make

.PHONY : project_generate_messages_py

# Rule to build all files generated by this target.
CMakeFiles/project_generate_messages_py.dir/build: project_generate_messages_py

.PHONY : CMakeFiles/project_generate_messages_py.dir/build

CMakeFiles/project_generate_messages_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/project_generate_messages_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/project_generate_messages_py.dir/clean

CMakeFiles/project_generate_messages_py.dir/depend:
	cd /home/ricca/catkin_ws/src/ROB-project/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ricca/catkin_ws/src/ROB-project /home/ricca/catkin_ws/src/ROB-project /home/ricca/catkin_ws/src/ROB-project/build /home/ricca/catkin_ws/src/ROB-project/build /home/ricca/catkin_ws/src/ROB-project/build/CMakeFiles/project_generate_messages_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/project_generate_messages_py.dir/depend
