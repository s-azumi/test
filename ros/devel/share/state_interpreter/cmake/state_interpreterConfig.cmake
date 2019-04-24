# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(state_interpreter_CONFIG_INCLUDED)
  return()
endif()
set(state_interpreter_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
if("TRUE" STREQUAL "TRUE")
  set(state_interpreter_SOURCE_PREFIX /home/azumi-suzuki/workspace/yaml_dsl/ros/src/state_interpreter)
  set(state_interpreter_DEVEL_PREFIX /home/azumi-suzuki/workspace/yaml_dsl/ros/devel)
  set(state_interpreter_INSTALL_PREFIX "")
  set(state_interpreter_PREFIX ${state_interpreter_DEVEL_PREFIX})
else()
  set(state_interpreter_SOURCE_PREFIX "")
  set(state_interpreter_DEVEL_PREFIX "")
  set(state_interpreter_INSTALL_PREFIX /home/azumi-suzuki/workspace/yaml_dsl/ros/install)
  set(state_interpreter_PREFIX ${state_interpreter_INSTALL_PREFIX})
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'state_interpreter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(state_interpreter_FOUND_CATKIN_PROJECT TRUE)

if(NOT " " STREQUAL " ")
  set(state_interpreter_INCLUDE_DIRS "")
  set(_include_dirs "")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'azumi-suzuki <azumi-suzuki@todo.todo>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${state_interpreter_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'state_interpreter' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
      message(FATAL_ERROR "Project 'state_interpreter' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/azumi-suzuki/workspace/yaml_dsl/ros/src/state_interpreter/${idir}'.  ${_report}")
    endif()
    _list_append_unique(state_interpreter_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND state_interpreter_LIBRARIES ${library})
  elseif(TARGET ${library})
    list(APPEND state_interpreter_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND state_interpreter_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
    foreach(path /home/azumi-suzuki/workspace/yaml_dsl/ros/devel/lib;/home/azumi-suzuki/autoware_azm/ros/install/ymc/lib;/home/azumi-suzuki/autoware_azm/ros/install/xsens_driver/lib;/home/azumi-suzuki/autoware_azm/ros/install/lattice_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/waypoint_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/waypoint_maker/lib;/home/azumi-suzuki/autoware_azm/ros/install/way_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/trafficlight_recognizer/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_utilities/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_simulation_package/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_local_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_global_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_kf_contour_track/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_ros_helpers/lib;/home/azumi-suzuki/autoware_azm/ros/install/lane_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/ff_waypoint_follower/lib;/home/azumi-suzuki/autoware_azm/ros/install/dp_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/waypoint_follower/lib;/home/azumi-suzuki/autoware_azm/ros/install/vlg22c_cam/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_ssd_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_segment_enet_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_lane_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_dpm_ttic_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_darknet_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vision_beyond_track/lib;/home/azumi-suzuki/autoware_azm/ros/install/vehicle_socket/lib;/home/azumi-suzuki/autoware_azm/ros/install/vehicle_model/lib;/home/azumi-suzuki/autoware_azm/ros/install/vehicle_gazebo_simulation_launcher/lib;/home/azumi-suzuki/autoware_azm/ros/install/vehicle_gazebo_simulation_interface/lib;/home/azumi-suzuki/autoware_azm/ros/install/vehicle_description/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_simu/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/op_utility/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_euclidean_cluster_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/vector_map_server/lib;/home/azumi-suzuki/autoware_azm/ros/install/road_occupancy_processor/lib;/home/azumi-suzuki/autoware_azm/ros/install/costmap_generator/lib;/home/azumi-suzuki/autoware_azm/ros/install/object_map/lib;/home/azumi-suzuki/autoware_azm/ros/install/naive_motion_predict/lib;/home/azumi-suzuki/autoware_azm/ros/install/map_file/lib;/home/azumi-suzuki/autoware_azm/ros/install/libvectormap/lib;/home/azumi-suzuki/autoware_azm/ros/install/imm_ukf_pda_track/lib;/home/azumi-suzuki/autoware_azm/ros/install/decision_maker/lib;/home/azumi-suzuki/autoware_azm/ros/install/vector_map/lib;/home/azumi-suzuki/autoware_azm/ros/install/vector_map_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/vectacam/lib;/home/azumi-suzuki/autoware_azm/ros/install/udon_socket/lib;/home/azumi-suzuki/autoware_azm/ros/install/tablet_socket/lib;/home/azumi-suzuki/autoware_azm/ros/install/runtime_manager/lib;/home/azumi-suzuki/autoware_azm/ros/install/mqtt_socket/lib;/home/azumi-suzuki/autoware_azm/ros/install/tablet_socket_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/state_machine_lib/lib;/home/azumi-suzuki/autoware_azm/ros/install/sound_player/lib;/home/azumi-suzuki/autoware_azm/ros/install/sick_lms5xx/lib;/home/azumi-suzuki/autoware_azm/ros/install/sick_ldmrs_tools/lib;/home/azumi-suzuki/autoware_azm/ros/install/sick_ldmrs_driver/lib;/home/azumi-suzuki/autoware_azm/ros/install/sick_ldmrs_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/sick_ldmrs_description/lib;/home/azumi-suzuki/autoware_azm/ros/install/rslidar_driver/lib;/home/azumi-suzuki/autoware_azm/ros/install/rslidar_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/points2image/lib;/home/azumi-suzuki/autoware_azm/ros/install/rosinterface/lib;/home/azumi-suzuki/autoware_azm/ros/install/rosbag_controller/lib;/home/azumi-suzuki/autoware_azm/ros/install/roi_object_filter/lib;/home/azumi-suzuki/autoware_azm/ros/install/range_vision_fusion/lib;/home/azumi-suzuki/autoware_azm/ros/install/pos_db/lib;/home/azumi-suzuki/autoware_azm/ros/install/points_preprocessor/lib;/home/azumi-suzuki/autoware_azm/ros/install/points_downsampler/lib;/home/azumi-suzuki/autoware_azm/ros/install/pixel_cloud_fusion/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_localizer/lib;/home/azumi-suzuki/autoware_azm/ros/install/pcl_omp_registration/lib;/home/azumi-suzuki/autoware_azm/ros/install/pc2_downsampler/lib;/home/azumi-suzuki/autoware_azm/ros/install/ouster_ros/lib;/home/azumi-suzuki/autoware_azm/ros/install/oculus_socket/lib;/home/azumi-suzuki/autoware_azm/ros/install/obj_db/lib;/home/azumi-suzuki/autoware_azm/ros/install/nmea_navsat/lib;/home/azumi-suzuki/autoware_azm/ros/install/ndt_tku/lib;/home/azumi-suzuki/autoware_azm/ros/install/ndt_gpu/lib;/home/azumi-suzuki/autoware_azm/ros/install/ndt_cpu/lib;/home/azumi-suzuki/autoware_azm/ros/install/multi_lidar_calibrator/lib;/home/azumi-suzuki/autoware_azm/ros/install/microstrain_driver/lib;/home/azumi-suzuki/autoware_azm/ros/install/memsic_imu/lib;/home/azumi-suzuki/autoware_azm/ros/install/marker_downsampler/lib;/home/azumi-suzuki/autoware_azm/ros/install/map_tools/lib;/home/azumi-suzuki/autoware_azm/ros/install/map_tf_generator/lib;/home/azumi-suzuki/autoware_azm/ros/install/log_tools/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_shape_estimation/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_point_pillars/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_naive_l_shape_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_fake_perception/lib;/home/azumi-suzuki/autoware_azm/ros/install/lidar_apollo_cnn_seg_detect/lib;/home/azumi-suzuki/autoware_azm/ros/install/libdpm_ttic/lib;/home/azumi-suzuki/autoware_azm/ros/install/lgsvl_simulator_bridge/lib;/home/azumi-suzuki/autoware_azm/ros/install/lgsvl_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/kvaser/lib;/home/azumi-suzuki/autoware_azm/ros/install/kitti_launch/lib;/home/azumi-suzuki/autoware_azm/ros/install/kitti_player/lib;/home/azumi-suzuki/autoware_azm/ros/install/kitti_box_publisher/lib;/home/azumi-suzuki/autoware_azm/ros/install/javad_navsat_driver/lib;/home/azumi-suzuki/autoware_azm/ros/install/integrated_viewer/lib;/home/azumi-suzuki/autoware_azm/ros/install/image_processor/lib;/home/azumi-suzuki/autoware_azm/ros/install/hokuyo/lib;/home/azumi-suzuki/autoware_azm/ros/install/graph_tools/lib;/home/azumi-suzuki/autoware_azm/ros/install/gnss_localizer/lib;/home/azumi-suzuki/autoware_azm/ros/install/gnss/lib;/home/azumi-suzuki/autoware_azm/ros/install/glviewer/lib;/home/azumi-suzuki/autoware_azm/ros/install/gazebo_world_description/lib;/home/azumi-suzuki/autoware_azm/ros/install/gazebo_imu_description/lib;/home/azumi-suzuki/autoware_azm/ros/install/gazebo_camera_description/lib;/home/azumi-suzuki/autoware_azm/ros/install/garmin/lib;/home/azumi-suzuki/autoware_azm/ros/install/freespace_planner/lib;/home/azumi-suzuki/autoware_azm/ros/install/fastvirtualscan/lib;/home/azumi-suzuki/autoware_azm/ros/install/detected_objects_visualizer/lib;/home/azumi-suzuki/autoware_azm/ros/install/decision_maker_panel/lib;/home/azumi-suzuki/autoware_azm/ros/install/dbw_mkz_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/data_preprocessor/lib;/home/azumi-suzuki/autoware_azm/ros/install/custom_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/calibration_publisher/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_health_checker/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_system_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_rviz_plugins/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_pointgrey_drivers/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_driveworks_interface/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_connector/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_camera_lidar_calibrator/lib;/home/azumi-suzuki/autoware_azm/ros/install/astar_search/lib;/home/azumi-suzuki/autoware_azm/ros/install/as/lib;/home/azumi-suzuki/autoware_azm/ros/install/amathutils_lib/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_launcher_rviz/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_launcher/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_driveworks_gmsl_interface/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_config_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_can_msgs/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_build_flags/lib;/home/azumi-suzuki/autoware_azm/ros/install/autoware_bag_tools/lib;/home/azumi-suzuki/autoware_azm/ros/install/adi_driver/lib;/opt/ros/kinetic/lib)
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(state_interpreter_LIBRARY_DIRS ${lib_path})
      list(APPEND state_interpreter_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'state_interpreter'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND state_interpreter_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(state_interpreter_EXPORTED_TARGETS "")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${state_interpreter_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "roscpp;sound_play;std_msgs;geometry_msgs;diagnostic_msgs;autoware_msgs;autoware_config_msgs")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 state_interpreter_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${state_interpreter_dep}_FOUND)
      find_package(${state_interpreter_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${state_interpreter_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(state_interpreter_INCLUDE_DIRS ${${state_interpreter_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(state_interpreter_LIBRARIES ${state_interpreter_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${state_interpreter_dep}_LIBRARIES})
  _list_append_deduplicate(state_interpreter_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(state_interpreter_LIBRARIES ${state_interpreter_LIBRARIES})

  _list_append_unique(state_interpreter_LIBRARY_DIRS ${${state_interpreter_dep}_LIBRARY_DIRS})
  list(APPEND state_interpreter_EXPORTED_TARGETS ${${state_interpreter_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${state_interpreter_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
