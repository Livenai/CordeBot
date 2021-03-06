# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/carlos/robocomp/components/CordeBot/component

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/carlos/robocomp/components/CordeBot/component

# Utility rule file for ICES__home_carlos_robocomp_components_CordeBot_component_src.

# Include the progress variables for this target.
include src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/progress.make

ICES__home_carlos_robocomp_components_CordeBot_component_src: src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating CommonBehavior.ice from /opt/robocomp/interfaces/IDSLs/CommonBehavior.idsl"
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/carlos/robocomp/components/CordeBot/component/src/CommonBehavior.ice
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/CommonBehavior.idsl CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating GenericBase.ice from /opt/robocomp/interfaces/IDSLs/GenericBase.idsl"
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/GenericBase.idsl /home/carlos/robocomp/components/CordeBot/component/src/GenericBase.ice
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/GenericBase.idsl GenericBase.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating InnerModelManager.ice from /opt/robocomp/interfaces/IDSLs/InnerModelManager.idsl"
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/InnerModelManager.idsl /home/carlos/robocomp/components/CordeBot/component/src/InnerModelManager.ice
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/InnerModelManager.idsl InnerModelManager.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating Laser.ice from /opt/robocomp/interfaces/IDSLs/Laser.idsl"
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/Laser.idsl /home/carlos/robocomp/components/CordeBot/component/src/Laser.ice
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/Laser.idsl Laser.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating OmniRobot.ice from /opt/robocomp/interfaces/IDSLs/OmniRobot.idsl"
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/OmniRobot.idsl /home/carlos/robocomp/components/CordeBot/component/src/OmniRobot.ice
	cd /home/carlos/robocomp/components/CordeBot/component/src && robocompdsl /opt/robocomp/interfaces/IDSLs/OmniRobot.idsl OmniRobot.ice
.PHONY : ICES__home_carlos_robocomp_components_CordeBot_component_src

# Rule to build all files generated by this target.
src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/build: ICES__home_carlos_robocomp_components_CordeBot_component_src

.PHONY : src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/build

src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/clean:
	cd /home/carlos/robocomp/components/CordeBot/component/src && $(CMAKE_COMMAND) -P CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/clean

src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/depend:
	cd /home/carlos/robocomp/components/CordeBot/component && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/carlos/robocomp/components/CordeBot/component /home/carlos/robocomp/components/CordeBot/component/src /home/carlos/robocomp/components/CordeBot/component /home/carlos/robocomp/components/CordeBot/component/src /home/carlos/robocomp/components/CordeBot/component/src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/ICES__home_carlos_robocomp_components_CordeBot_component_src.dir/depend

