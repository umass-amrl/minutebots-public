cmake_minimum_required(VERSION 2.8.11)

PROJECT(robocup-ssl)

# Flag for toggling test compilation on or off.
OPTION(test "Build all tests." ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/third_party/")

#===============================================================================
# Targets selection
OPTION(SWARM_TARGETS "Compile only targets necessary for SWARM tests" FALSE)
#===============================================================================


INCLUDE(FindProtobuf)

FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(Protobuf REQUIRED)
FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
FIND_PACKAGE(LuaJIT REQUIRED)

#FIND_PACKAGE(Ceres REQUIRED PATHS "${CMAKE_MODULE_PATH}ceres-solver-1.12.0")

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE (FindRE2)
FIND_PACKAGE (RE2 REQUIRED)
# INCLUDE (FINDZ3)
FIND_PACKAGE (Z3 REQUIRED)
# INCLUDE (FindSRTR)
# FIND_PACKAGE (SRTR REQUIRED)
INCLUDE(${QT_USE_FILE})

SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Wsign-compare -Werror -g")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -rdynamic")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -openmp -fno-builtin-malloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-calloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-realloc")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fno-builtin-free -O3 -DNDEBUG")
  #SET(BUILD_SPECIFIC_LIBRARIES "tcmalloc")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O1")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

SET(libs
    ${PROTOBUF_LIBRARY} ${Z3_LIBRARIES} glog pthread atomic popt X11 ${BUILD_SPECIFIC_LIBRARIES} amrl-shared-lib)

FILE(GLOB PROTO_FILES src/proto/*.proto)

PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})

# If (NOT SWARM_TARGETS)
PROTOBUF_GENERATE_PYTHON(PROTO_PY ${PROTO_FILES})
# ENDIF (NOT SWARM_TARGETS)

INCLUDE_DIRECTORIES(src/)
INCLUDE_DIRECTORIES(third_party/SRTR/src/)
INCLUDE_DIRECTORIES(third_party/amrl_shared_lib/)
INCLUDE_DIRECTORIES(third_party/)
INCLUDE_DIRECTORIES(${PROTOBUF_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${LUAJIT_INCLUDE_DIR})
add_subdirectory(third_party/SRTR/)
ADD_SUBDIRECTORY(third_party/amrl_shared_lib/)

INCLUDE_DIRECTORIES(${RE2_INCLUDE_DIR})
# INCLUDE_DIRECTORIES(${SRTR_INCLUDE_DIR})
if (NOT SWARM_TARGETS)
ADD_LIBRARY(protobuf_library ${PROTO_SRCS} ${PROTO_PY})
ENDIF (NOT SWARM_TARGETS)
#SET(PROTO_SRCS /home/jaholtz/installs/include/google/protobuf)
if (SWARM_TARGETS)
ADD_LIBRARY(protobuf_library ${PROTO_SRCS})
ENDIF (SWARM_TARGETS)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(protobuf_library ${libs})

# ADD_LIBRARY(srtr_library STATIC IMPORTED)
# SET_TARGET_PROPERTIES(srtr_library PROPERTIES IMPORTED_LOCATION src/libsrtr_library.a)

ADD_LIBRARY(state_machine_libs
            third_party/SRTR/src/srtr/state_machine.cc)

TARGET_LINK_LIBRARIES(state_machine_libs protobuf_library ${libs})

# ADD_LIBRARY(srtr_library
#             src/SRTR/src/srtr/srtr.cc)
#
# TARGET_LINK_LIBRARIES(srtr_library protobuf_library ${libs})

ADD_LIBRARY(configuration_reader_library
            src/configuration_reader/config_types/config_interface.cc
            src/configuration_reader/config_types/config_double.cc
            src/configuration_reader/config_types/config_float.cc
            src/configuration_reader/config_types/config_int.cc
            src/configuration_reader/config_types/config_string.cc
            src/configuration_reader/config_types/config_unsigned_int.cc
            src/configuration_reader/config_types/config_vector2f.cc
            src/configuration_reader/lua_script.cc
            src/configuration_reader/reader.cc)
TARGET_LINK_LIBRARIES(configuration_reader_library ${libs} ${LUAJIT_LIBRARIES})

ADD_LIBRARY(constants_library
            src/constants/constants.cc)
TARGET_LINK_LIBRARIES(constants_library protobuf_library configuration_reader_library ${RE2_LIBRARIES} ${libs})

ADD_LIBRARY(net_library
  src/net/netraw.cc)
TARGET_LINK_LIBRARIES(net_library protobuf_library ${RE2_LIBRARIES} ${libs})

ADD_LIBRARY(team_library
  src/state/team.cc)
TARGET_LINK_LIBRARIES(team_library protobuf_library ${RE2_LIBRARIES} ${libs})

ADD_LIBRARY(thread_safe_library
  src/thread_safe/thread_safe_actor.h
  src/thread_safe/thread_safe_priority_queue.h
  src/thread_safe/thread_safe_queue.h)
TARGET_LINK_LIBRARIES(thread_safe_library protobuf_library ${RE2_LIBRARIES} ${libs})

ADD_EXECUTABLE(netraw_tester src/net/netraw_tester.cc)
TARGET_LINK_LIBRARIES(netraw_tester constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(test_view_callback src/gui/test_view_callback.cc)
TARGET_LINK_LIBRARIES(test_view_callback constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(netraw_tester_large src/net/netraw_tester_large.cc)
TARGET_LINK_LIBRARIES(netraw_tester_large constants_library net_library team_library amrl-shared-lib ${libs})

ADD_LIBRARY(zone_library
  src/zone/zone.cc
)

ADD_LIBRARY(obstacle_library
  src/obstacles/obstacle.cc
  src/obstacles/obstacle_flag.cc
  src/obstacles/circle_obstacle.cc
  src/obstacles/rectangle_obstacle.cc
  src/obstacles/ball_obstacle.cc
  src/obstacles/robot_obstacle.cc
  src/obstacles/safety_margin.cc
  src/obstacles/integer_bounding_box.cc)
TARGET_LINK_LIBRARIES(obstacle_library constants_library net_library team_library amrl-shared-lib main_library ${libs})

ADD_LIBRARY(graph_library
  src/graph/fastmultigraph.cc
  src/graph/graph_util.cc
  src/graph/graph.cc
  src/graph/edge.cc
  src/graph/vertex.cc)
  TARGET_LINK_LIBRARIES(graph_library obstacle_library constants_library net_library team_library amrl-shared-lib protobuf_library
  ${libs})

ADD_LIBRARY(third_party_library
  third_party/RadixHeap/radixheap.cc)
TARGET_LINK_LIBRARIES(third_party_library ${libs})

ADD_LIBRARY(navigation_library
  src/navigation/production/production_eight_grid.cc
  src/navigation/production/collision_grid.cc
  src/navigation/production/eight_grid_common_utils.cc
  src/navigation/lazy_PRM.cc
  src/navigation/PRM.cc
  src/navigation/RRT.cc
  src/navigation/navigation.cc
  src/navigation/repair/multipath_collision_checks.cc
  src/navigation/eight_grid.cc
  src/navigation/navigation_util.cc
  src/navigation/scaffolding/scaffold.cc
  src/navigation/static_tangent_planner.cc
  src/navigation/stox_planner.cc)
TARGET_LINK_LIBRARIES(navigation_library third_party_library
graph_library obstacle_library constants_library net_library team_library amrl-shared-lib protobuf_library
  ${libs})

ADD_LIBRARY(motion_model_library
  src/state_estimation/motion_model.cc
  src/state_estimation/default_motion_model.cc)

TARGET_LINK_LIBRARIES(motion_model_library logging_library constants_library net_library team_library amrl-shared-lib
                      ${libs})

ADD_LIBRARY(state_estimation_library
  src/state_estimation/chip_kick_detection.cc
  src/state_estimation/chip_kick_tester.cc
  src/state_estimation/extended_kalman_filter.cc
  src/state_estimation/ball_state_estimator.cc
  src/state_estimation/KF_const_accel.cc
  src/state_estimation/KF_const_vel.cc)
TARGET_LINK_LIBRARIES(state_estimation_library motion_model_library
                      logging_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_LIBRARY(main_library
  src/soccer/kalmanupdate.cc
  src/soccer/executor.cc
  src/experimental_sim/experimental_sim.cc
  src/evaluators/offense_evaluation.cc
  src/evaluators/stopped_evaluation.cc
  src/evaluators/penalty_recieve_evaluation.cc
  src/evaluators/defense_evaluation.cc
  src/evaluators/field_visualizer.cc
  src/evaluators/passing_evaluation.cc)
TARGET_LINK_LIBRARIES(main_library navigation_library protobuf_library
                      experimental_sim_library X11
                      safety_library obstacle_library constants_library net_library team_library amrl-shared-lib
                      navigation_logging_library
                      state_estimation_library motion_model_library ${libs})

ADD_LIBRARY(state_library
            src/motion_control/ball_interception.cc
            src/state/world_robot.cc
            src/state/world_state.cc
            src/state/position_velocity_state.cc
            src/state/world_ball.cc
            src/state/soccer_robot.cc
            src/state/soccer_state.cc
            src/state/shared_state.cc
            src/state/referee_state.cc
            src/obstacles/robot_obstacle.cc)
TARGET_LINK_LIBRARIES(state_library main_library motion_model_library
                      obstacle_library constants_library net_library team_library amrl-shared-lib
                      protobuf_library ${libs})

ADD_LIBRARY(logging_library
            src/state/referee_state.cc
            src/logging/score_episode_logger.cc
            src/logging/logger.cc)
TARGET_LINK_LIBRARIES(logging_library main_library constants_library net_library team_library amrl-shared-lib
                      protobuf_library state_library ${libs})

ADD_LIBRARY(navigation_logging_library
            src/logging/navigation_logger.cc)
TARGET_LINK_LIBRARIES(navigation_logging_library protobuf_library
                      constants_library net_library team_library amrl-shared-lib ${libs})

ADD_LIBRARY(motion_library
  src/motion_control/optimal_control_1d.cc
  src/motion_control/ntoc_2d.cc
  src/motion_control/tsocs_old.cc
  src/motion_control/tsocs_alt.cc
  src/motion_control/ball_interception.cc
  src/motion_control/sub_optimal_controls.cc
  src/motion_control/deflection_controller.cc
  src/motion_control/kinematic_model.cc
  src/motion_control/ball_interception2.cc
  src/motion_control/angular_planner.cc
  src/motion_control/tsocs.cc)
TARGET_LINK_LIBRARIES(motion_library state_library constants_library net_library team_library amrl-shared-lib
  ${CERES_LIBRARIES} ${libs})

ADD_LIBRARY(joystick_library
  src/joystick/joystick.cc
)
TARGET_LINK_LIBRARIES(joystick_library constants_library net_library team_library amrl-shared-lib protobuf_library ${libs})

ADD_LIBRARY(robocup_eastar_library
  src/search/robocup_eastar/common_defines.cc
  src/search/robocup_eastar/eastar_solver.cc
  src/search/robocup_eastar/individual_planner.cc
  src/search/robocup_eastar/path_collision_detector.cc
  src/search/robocup_eastar/multiagent_data.cc
  src/search/robocup_eastar/search_window.cc
  src/search/robocup_eastar/planner_state.cc
  src/search/robocup_eastar/lazy_neighbor_generator.cc
  src/navigation/production/collision_grid.cc
  src/navigation/production/eight_grid_common_utils.cc
  src/navigation/production/production_eight_grid.cc)

TARGET_LINK_LIBRARIES(robocup_eastar_library
                      main_library
                      constants_library net_library team_library amrl-shared-lib
                      logging_library
                      state_library
                      navigation_library
                      motion_library
                      tactic_library
                      protobuf_library
                      obstacle_library
                      state_library
                      play_library ${libs})

ADD_EXECUTABLE(robocup_eastar
  src/search/robocup_eastar/eastar_main.cc)
TARGET_LINK_LIBRARIES(robocup_eastar
                      robocup_eastar_library
                      main_library
                      constants_library net_library team_library amrl-shared-lib
                      logging_library
                      state_library
                      navigation_library
                      motion_library
                      tactic_library
                      protobuf_library
                      obstacle_library
                      state_library
                      play_library
                      ${libs})

ADD_LIBRARY(mstar_library
  third_party/mstar/col_checker.cpp
  third_party/mstar/epemstar.cpp
  third_party/mstar/grid_planning.cpp
  third_party/mstar/grid_policy.cpp
  third_party/mstar/mstar_utils.cpp
  third_party/mstar/od_mstar.cpp
  third_party/mstar/policy.cpp)

ADD_EXECUTABLE(robocup_mstar
  src/search/robocup_mstar/mstar_main.cc)
TARGET_LINK_LIBRARIES(robocup_mstar
                      mstar_library
                      robocup_eastar_library
                      main_library
                      constants_library net_library team_library amrl-shared-lib
                      logging_library
                      state_library
                      navigation_library
                      motion_library
                      tactic_library
                      protobuf_library
                      obstacle_library
                      state_library
                      play_library
                      ${libs})

ADD_EXECUTABLE(robocup_cbs
  src/search/robocup_cbs/cbs_main.cc)
TARGET_LINK_LIBRARIES(robocup_cbs
                      robocup_eastar_library
                      main_library
                      constants_library net_library team_library amrl-shared-lib
                      logging_library
                      state_library
                      navigation_library
                      motion_library
                      tactic_library
                      protobuf_library
                      obstacle_library
                      state_library
                      play_library
                      ${libs})

ADD_LIBRARY(fourgrid_library
  src/search/fourgrid/fourgrid.cc
  src/search/fourgrid/fourgrid_solver.cc
  src/search/fourgrid/dstar_fourgrid_solver.cc)
TARGET_LINK_LIBRARIES(fourgrid_library logging_library ${libs})

ADD_EXECUTABLE(fourgrind_individual_to_joint src/search/fourgrid/fourgrid_individual_to_joint.cc)
TARGET_LINK_LIBRARIES(fourgrind_individual_to_joint
    fourgrid_library
    logging_library)

ADD_EXECUTABLE(fourgrind_individual_to_individual src/search/fourgrid/fourgrid_individual_to_individual.cc)
TARGET_LINK_LIBRARIES(fourgrind_individual_to_individual
    fourgrid_library
    logging_library)

# ADD_EXECUTABLE(fourgrind_dual_robot src/search/fourgrid/fourgrid_dual_robot.cc)
# TARGET_LINK_LIBRARIES(fourgrind_dual_robot
#     fourgrid_library
#     logging_library)
#
# ADD_EXECUTABLE(fourgrind_single_robot src/search/fourgrid/fourgrid_single_robot.cc)
# TARGET_LINK_LIBRARIES(fourgrind_single_robot
#     fourgrid_library
#     logging_library)

ADD_LIBRARY(tactic_library
            src/tactics/tactic.cc
            src/tactics/state_machine_tactic.cc
            src/tactics/eight_grid_navigation.cc
            src/tactics/coordinates.cc
            src/tactics/forward_backward.cc
            src/tactics/triangle.cc
            src/tactics/triangle_id_dependent.cc
            src/tactics/goalie.cc
            src/tactics/primary_defender.cc
            src/tactics/simple_attacker.cc
            src/tactics/secondary_defender.cc
            src/tactics/guard_point.cc
            src/tactics/halt.cc
            src/tactics/ntoc_controller.cc
            src/tactics/joystick_controller.cc
            src/tactics/kickoff_kicker.cc
            src/tactics/penalty_kicker.cc
            src/tactics/direct_free_kicker.cc
            src/tactics/indirect_free_kicker.cc
            src/tactics/ball_interception.cc
            src/tactics/catch.cc
            src/tactics/stox_pivot.cc
            src/tactics/custom_route.cc
            src/joystick/joystick.cc
            src/tactics/random_points.cc
            src/tactics/test_catch.cc
            src/tactics/safe_follow_ball.cc
            src/tactics/penalty_recieve.cc
            src/tactics/navigate_to_intercept.cc
            src/tactics/navigate_to_catch.cc
            src/tactics/secondary_attacker.cc
            src/tactics/dive_controller.cc
            src/tactics/test_dive_controller.cc
            src/tactics/kick.cc
            src/tactics/three_kick.cc
            src/tactics/primary_attacker.cc
            src/tactics/example_attacker.cc
            src/tactics/state_machine_example.cc
            src/tactics/ball_placement.cc
            src/tactics/receiver.cc
            src/tactics/test_passing.cc
            src/tactics/test_passback.cc
            src/tactics/setup_attacker.cc
            src/tactics/coercive_attacker.cc
            src/tactics/test_ntoc.cc
            src/tactics/deflection.cc
            src/tactics/SimpleDeflection.cc
            src/tactics/pd_controller.cc
            src/tactics/tertiary_defender.cc
            src/tactics/test_defender.cc
            src/tactics/test_attacker.cc
            src/tactics/better_ball_placement.cc
            src/tactics/docking.cc
            src/tactics/tsocs_controller.cc
            src/tactics/test_tsocs.cc
            src/tactics/demo_tsocs.cc
            src/tactics/train_tsocs.cc
            src/tactics/test_sim_tsocs.cc
            src/tactics/marionette.cc)

TARGET_LINK_LIBRARIES(tactic_library navigation_logging_library main_library
                      constants_library net_library team_library amrl-shared-lib navigation_library
                      state_machine_libs
                      configuration_reader_library protobuf_library
                      motion_library zone_library ${libs})

ADD_LIBRARY(play_library
            src/plays/skills_tactics_plays.cc
            src/plays/predicates.cc)
TARGET_LINK_LIBRARIES(play_library main_library constants_library net_library team_library amrl-shared-lib
                      protobuf_library ${libs})

ADD_EXECUTABLE(soccer src/soccer/soccer_main.cc)
TARGET_LINK_LIBRARIES(soccer main_library protobuf_library
                      experimental_sim_library configuration_reader_library
                      obstacle_library navigation_library constants_library net_library team_library amrl-shared-lib
                      state_library tactic_library play_library state_estimation_library
                      logging_library joystick_library
                      ${libs} ${LUAJIT_LIBRARIES})

ADD_EXECUTABLE(episode_logger src/logging/episode_logger_main.cc)
TARGET_LINK_LIBRARIES(episode_logger main_library protobuf_library
                      experimental_sim_library configuration_reader_library
                      obstacle_library navigation_library constants_library net_library team_library amrl-shared-lib
                      state_library tactic_library play_library
                      state_estimation_library
                      logging_library joystick_library
                      ${libs} ${LUAJIT_LIBRARIES})

ADD_EXECUTABLE(dss_tester src/safety/dss_tester.cc)
TARGET_LINK_LIBRARIES(dss_tester main_library protobuf_library
                      obstacle_library navigation_library constants_library net_library team_library amrl-shared-lib
                      state_library tactic_library play_library
                      state_estimation_library
                      logging_library joystick_library
                      ${libs} ${LUAJIT_LIBRARIES})

ADD_LIBRARY(simulator_library
  src/sim/simulator.cc
  src/sim/networkhandler.cc
  src/sim/worldstate.cc
  src/sim/simulator.cc
  src/sim/objects/ball.cc
  src/sim/objects/robot.cc)
TARGET_LINK_LIBRARIES(simulator_library constants_library net_library team_library amrl-shared-lib protobuf_library thread_safe_library ${libs})

ADD_LIBRARY(experimental_sim_library
  src/experimental_sim/networkhandler.cc
  src/experimental_sim/simulator.cc
  src/experimental_sim/sim_state.cc
  src/experimental_sim/experimental_sim.cc
  src/experimental_sim/objects/ball.cc
  src/experimental_sim/objects/robot.cc
  src/experimental_sim/sim_motion_model.cc
  src/experimental_sim/objects/robot.cc)
TARGET_LINK_LIBRARIES(experimental_sim_library obstacle_library constants_library net_library team_library amrl-shared-lib protobuf_library ${libs})

ADD_LIBRARY(simplecar_library
  src/simplecar_sim/simplecar.cc
  src/simplecar_sim/simplecar_state.cc
  src/simplecar_sim/simplecar_sim.cc
  src/simplecar_sim/simplecar_merge.cc
  src/simplecar_sim/simplecar_control.cc)

ADD_EXECUTABLE(simplecar src/simplecar_sim/simplecar_main.cc)
TARGET_LINK_LIBRARIES(simplecar simplecar_library logging_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(merge_simplecar src/simplecar_sim/simplecar_main_merge.cc)
TARGET_LINK_LIBRARIES(merge_simplecar simplecar_library logging_library amrl-shared-lib ${libs})

ADD_LIBRARY(deflection_sim_library
  src/deflection_sim/networkhandler.cc
  src/deflection_sim/worldstate.cc
  src/deflection_sim/objects/ball.cc
  src/deflection_sim/objects/robot.cc)
TARGET_LINK_LIBRARIES(deflection_sim_library constants_library net_library team_library amrl-shared-lib thread_safe_library protobuf_library  ${libs})

ADD_LIBRARY(safety_library
  src/safety/dss.cc
  src/safety/dss_helpers.cc
  src/safety/dss2.cc)
TARGET_LINK_LIBRARIES(safety_library constants_library net_library team_library amrl-shared-lib state_library tactic_library
  protobuf_library ${libs})


ADD_EXECUTABLE(collect_TSOCS_data src/motion_control/CollectTSOCSdata.cc)
TARGET_LINK_LIBRARIES(collect_TSOCS_data constants_library net_library team_library amrl-shared-lib motion_library)

ADD_EXECUTABLE(generate_intercept_data
                src/motion_control/test_intercept_convergence.cc)
TARGET_LINK_LIBRARIES(generate_intercept_data constants_library net_library team_library amrl-shared-lib motion_library)

ADD_EXECUTABLE(generate_control_stability_data
                src/motion_control/stability_data_generator.cc)
TARGET_LINK_LIBRARIES(generate_control_stability_data constants_library net_library team_library amrl-shared-lib
                      motion_library)

ADD_EXECUTABLE(navigation_tester src/navigation/navigation_tester.cc)
TARGET_LINK_LIBRARIES(navigation_tester navigation_library state_library
constants_library net_library team_library amrl-shared-lib ${libs})

qt4_wrap_cpp(GUI_MOC_SRCS
  src/gui/viewer_main.h
  src/gui/graph.h
  src/gui/soccerview.h
  src/gui/text_tree_viewer.h
  src/gui/viewer.h
)

qt4_wrap_cpp(CARGUI_MOC_SRCS
  src/simplecar_sim/simplecar_viewer_main.h
  src/gui/graph.h
  src/simplecar_sim/simplecar_view.h
  src/gui/text_tree_viewer.h
  src/simplecar_sim/simplecar_viewer.h
  src/gui/viewer.h
  src/gui/soccerview.h
)

ADD_LIBRARY(gui_library
  src/gui/drawing_helpers.cc
  src/gui/gltext.cc
  src/gui/graph.cc
  src/gui/opengl_helpers.cc
  src/gui/soccerview.cc
  src/gui/text_tree_viewer.cc
  src/gui/viewer.cc
  ${GUI_MOC_SRCS}
)
TARGET_LINK_LIBRARIES(gui_library ${QT_LIBRARIES} constants_library net_library team_library amrl-shared-lib logging_library ${libs})


IF (NOT SWARM_TARGETS)
ADD_EXECUTABLE(viewer src/gui/viewer_main.cc)
TARGET_LINK_LIBRARIES(viewer
  gui_library
  constants_library net_library team_library amrl-shared-lib
  logging_library
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${libs}
)

ADD_LIBRARY(simplecar_gui_library
  src/gui/drawing_helpers.cc
  src/gui/gltext.cc
  src/gui/graph.cc
  src/gui/opengl_helpers.cc
  src/simplecar_sim/simplecar_view.cc
  src/gui/text_tree_viewer.cc
  src/gui/viewer.cc
  src/simplecar_sim/simplecar_viewer.cc
  src/gui/soccerview.cc
  ${CARGUI_MOC_SRCS}
)
TARGET_LINK_LIBRARIES(simplecar_gui_library ${QT_LIBRARIES} constants_library net_library team_library amrl-shared-lib logging_library ${libs})

ADD_EXECUTABLE(carviewer src/simplecar_sim//simplecar_viewer_main.cc)
TARGET_LINK_LIBRARIES(carviewer
  simplecar_gui_library
  constants_library net_library team_library amrl-shared-lib
  logging_library
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${libs}
)
ENDIF (NOT SWARM_TARGETS)


qt4_wrap_cpp(VIEWER_MOTION_MOC_SRCS
  src/gui/viewer_motion.h
)

ADD_EXECUTABLE(viewer_motion
 src/gui/viewer_motion.cc
 ${VIEWER_MOTION_MOC_SRCS}
)
TARGET_LINK_LIBRARIES(viewer_motion
 gui_library
 constants_library net_library team_library amrl-shared-lib
 motion_library
 X11
 ${OPENGL_gl_LIBRARY}
 ${OPENGL_glu_LIBRARY}
 ${libs}
)

qt4_wrap_cpp(VIEWER_STATE_MOC_SRCS
  src/gui/viewer_state.h
)

ADD_EXECUTABLE(viewer_state
 src/gui/viewer_state.cc
 ${VIEWER_STATE_MOC_SRCS}
)
TARGET_LINK_LIBRARIES(viewer_state
 gui_library
 constants_library net_library team_library amrl-shared-lib
 motion_library
 X11
 ${OPENGL_gl_LIBRARY}
 ${OPENGL_glu_LIBRARY}
 ${libs}
)

# ADD_EXECUTABLE(kalman_triangle
#   src/gui/triangle_integration_viewer.cc
# )

# TARGET_LINK_LIBRARIES(kalman_triangle
#   gui_library
#   constants_library net_library team_library amrl-shared-lib
#   motion_library
#   state_estimation_library
#   ${OPENGL_gl_LIBRARY}
#   ${OPENGL_glu_LIBRARY}
#   ${libs}
# )

ADD_EXECUTABLE(text_tree_viewer_tester src/gui/text_tree_viewer_tester.cc)
TARGET_LINK_LIBRARIES(text_tree_viewer_tester
  gui_library
  constants_library net_library team_library amrl-shared-lib
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${libs}
)

ADD_EXECUTABLE(soccerview_tester src/gui/soccerview_tester.cc)
TARGET_LINK_LIBRARIES(soccerview_tester
  gui_library
  constants_library net_library team_library amrl-shared-lib
  ${OPENGL_gl_LIBRARY}
  ${OPENGL_glu_LIBRARY}
  ${libs}
)

ADD_EXECUTABLE(z3_tuner src/magic_debugging/tuning_sketch.cpp)
TARGET_LINK_LIBRARIES(z3_tuner
logging_library
gui_library
constants_library net_library team_library amrl-shared-lib
srtr_library
protobuf_library
)

ADD_EXECUTABLE(trace_from_log src/magic_debugging/trace_from_log.cpp)
TARGET_LINK_LIBRARIES(trace_from_log
logging_library
gui_library
constants_library net_library team_library amrl-shared-lib
srtr_library
protobuf_library
)

ADD_EXECUTABLE(srtr_brass src/magic_debugging/srtr_brass.cpp)
TARGET_LINK_LIBRARIES(srtr_brass
logging_library
gui_library
constants_library net_library team_library amrl-shared-lib
srtr_library
protobuf_library
)

ADD_EXECUTABLE(gen_corrections src/magic_debugging/generate_brass_corrections.cpp)
TARGET_LINK_LIBRARIES(gen_corrections
logging_library
gui_library
constants_library net_library team_library amrl-shared-lib
srtr_library
protobuf_library
)

ADD_EXECUTABLE(joystick_driver
  src/joystick/joystick_driver.cc)
TARGET_LINK_LIBRARIES(joystick_driver joystick_library yisibot_radio_library
constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(joystick_driver_logging
  src/open_loop_executors/open_loop_joystick_main.cc
  src/open_loop_executors/open_loop_joystick.cc)
TARGET_LINK_LIBRARIES(joystick_driver_logging
  joystick_library state_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(chip_kick_tuner
  src/open_loop_executors/chip_kick_tuner_main.cc)
TARGET_LINK_LIBRARIES(chip_kick_tuner
  joystick_library state_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(flat_kick_tuner
  src/open_loop_executors/flat_kick_tuner_main.cc
  src/open_loop_executors/ball_reader.cc)
TARGET_LINK_LIBRARIES(flat_kick_tuner
  joystick_library state_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(joystick_referee_driver
src/joystick/joystick_referee_driver.cc)
TARGET_LINK_LIBRARIES(joystick_referee_driver joystick_library
yisibot_radio_library
constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(ramp_controller
               src/open_loop_executors/ramp_controller_main.cc
               src/open_loop_executors/camera_reader.cc
               src/open_loop_executors/ramp_controller.cc)
TARGET_LINK_LIBRARIES(ramp_controller state_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(sine_controller
               src/open_loop_executors/sine_controller_main.cc
               src/open_loop_executors/camera_reader.cc
               src/open_loop_executors/sine_controller.cc)
TARGET_LINK_LIBRARIES(sine_controller state_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(simulator src/sim/sim_main.cc)
TARGET_LINK_LIBRARIES(simulator simulator_library protobuf_library
   obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(experimental_sim src/experimental_sim/sim_main.cc)
TARGET_LINK_LIBRARIES(experimental_sim experimental_sim_library protobuf_library
   obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(test_simulator src/sim/simulator_tester.cc)
TARGET_LINK_LIBRARIES(test_simulator simulator_library protobuf_library
obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(experimental_test src/experimental_sim/simulator_tester.cc)
TARGET_LINK_LIBRARIES(experimental_test experimental_sim_library protobuf_library
obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

# ADD_EXECUTABLE(factor_tuning src/magic_debugging/factor_tuning.cc)
# TARGET_LINK_LIBRARIES(factor_tuning bespoke_sim_library protobuf_library
# obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(deflection_sim src/deflection_sim/deflection_sim_main.cc)
TARGET_LINK_LIBRARIES(deflection_sim deflection_sim_library protobuf_library
obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

# ADD_EXECUTABLE(attacker_test src/deflection_sim/attacker_test.cc)
# TARGET_LINK_LIBRARIES(attacker_test deflection_sim_library protobuf_library tactic_library
# obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

# ADD_EXECUTABLE(deflection_test src/deflection_sim/deflection_test.cc)
# TARGET_LINK_LIBRARIES(deflection_test deflection_sim_library protobuf_library tactic_library
# obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

# ADD_EXECUTABLE(docking_test src/deflection_sim/docking_test.cc)
# TARGET_LINK_LIBRARIES(docking_test deflection_sim_library protobuf_library tactic_library
# obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(debugging_tester src/debugging/debugging_tester.cc)
TARGET_LINK_LIBRARIES(debugging_tester constants_library net_library team_library amrl-shared-lib ${libs})

ADD_EXECUTABLE(radio_module_tester src/radio/radio_module_tester.cc)
TARGET_LINK_LIBRARIES(radio_module_tester
    yisibot_radio_library
    constants_library net_library team_library amrl-shared-lib
    ${libs})

ADD_EXECUTABLE(motor_controller_graph src/hardware/motor_controller_graph.cc)
TARGET_LINK_LIBRARIES(motor_controller_graph
    yisibot_radio_library
    constants_library net_library team_library amrl-shared-lib
    ${libs})

ADD_EXECUTABLE(direct_drive_tester src/radio/direct_drive_tester.cc)
TARGET_LINK_LIBRARIES(direct_drive_tester
    yisibot_radio_library
    joystick_library
    constants_library net_library team_library amrl-shared-lib
    ${libs})

# ADD_EXECUTABLE(expanding_astar src/search/expanding_astar/expanding_main.cc
#  src/search/expanding_astar/expanding_astar_solver.cc
#   src/graph/graph.cc
#   src/obstacles/obstacle_flag.cc
#   src/obstacles/safety_margin.cc
#   src/graph/vertex.cc
#   src/graph/edge.cc
#   src/constants/constants.cc)
# TARGET_LINK_LIBRARIES(expanding_astar
#     logging_library)
#
# ADD_EXECUTABLE(expanding_astar_one src/search/expanding_astar/expanding_main_one.cc
#  src/search/expanding_astar/expanding_astar_solver.cc
#   src/graph/graph.cc
#   src/obstacles/obstacle_flag.cc
#   src/obstacles/safety_margin.cc
#   src/graph/vertex.cc
#   src/graph/edge.cc
#   src/constants/constants.cc)
# TARGET_LINK_LIBRARIES(expanding_astar_one
#     logging_library)

IF (test)
  INCLUDE_DIRECTORIES(third_party/)
  # This adds the googletest subdirectory, which has 'project(gtest)'.
  ADD_SUBDIRECTORY(third_party/googletest/googletest-release-1.8.0/googletest)

  ENABLE_TESTING()

  # Include the gtest library. gtest_SOURCE_DIR is available due to
  # 'project(gtest)' above.
  INCLUDE_DIRECTORIES(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

  # Main Unit Test executable.
  #  src/test/navigation/PRM_tests.cc
  ADD_EXECUTABLE(run_unit_tests src/test/test_main.cc
    src/test/obstacles/obstacle_tests.cc
    src/test/navigation/eight_grid_tests.cc
    src/test/navigation/production/production_eight_grid_tests.cc
    src/test/navigation/repair/multipath_collision_checks_tests.cc
    src/test/navigation/PRM_tests.cc
    src/test/navigation/RRT_tests.cc
    src/test/navigation/navigation_util_tests.cc
    src/test/evaluators/offense_tests.cc
    src/test/evaluators/defense_tests.cc
    src/test/math/geometry_tests.cc
    src/test/math/math_tests.cc
    src/test/motion_control/motion_control_test.cc
    src/test/safety/dss_tests.cc
    src/test/state/state_tests.cc
    src/test/search/fourgrid/fourgrid_solver_tests.cc
    src/test/search/robocup_eastar/lazy_neighbor_generator_tests.cc
    src/test/state_estimation/state_estimation_tests.cc
    src/test/datastructures/datastructures_test.cc
    src/test/logging/logging_tests.cc
    src/test/radio/radio_tests.cc
    src/test/thread_safe/thread_safe_tests.cc
    src/test/navigation/scenario_generator.cc)

  # Standard linking to gtest stuff.
  TARGET_LINK_LIBRARIES(run_unit_tests gtest gtest_main)

  # Extra linking for the project.
  TARGET_LINK_LIBRARIES(run_unit_tests
    constants_library net_library team_library amrl-shared-lib
    safety_library
    state_library
    main_library
    motion_library
    navigation_library
    fourgrid_library
    robocup_eastar_library
    simulator_library
    yisibot_radio_library
    logging_library)

  ADD_EXECUTABLE(run_performance_tests src/test/performance_test_main.cc
    src/test/navigation/tangent_planner_performance_tests.cc
    src/test/navigation/PRM_performance_tests.cc
    src/test/navigation/RRT_performance_tests.cc
    src/test/navigation/scenario_generator.cc
    src/test/navigation/repair/eight_grid_repairer_demo_tests.cc)
  TARGET_LINK_LIBRARIES(run_performance_tests gtest gtest_main
    navigation_library protobuf_library obstacle_library constants_library net_library team_library amrl-shared-lib ${libs})

  #ADD_EXECUTABLE(radio_server
  #            src/radio/radio_server_main.cc
  #            src/radio/networkhandler.cc)
  #TARGET_LINK_LIBRARIES(radio_server main_library constants_library net_library team_library amrl-shared-lib ${libs})

  ADD_LIBRARY(yisibot_radio_library
              src/yisibot_radio/yisibot_radio.cc
              src/yisibot_radio/crc.cc
              src/yisibot_radio/serial.cc
              src/radio/radio_helper.cc)
  TARGET_LINK_LIBRARIES(yisibot_radio_library protobuf_library ${libs})

  ADD_EXECUTABLE(yisibot_radio_server
  src/yisibot_radio/yisibot_radio_server_main.cc
  src/yisibot_radio/networkhandler.cc)
  TARGET_LINK_LIBRARIES(yisibot_radio_server yisibot_radio_library main_library
  constants_library net_library team_library amrl-shared-lib protobuf_library ${libs})

  ADD_LIBRARY(minutebot_radio_library
              src/radio/minutebot_radio.cc
              src/radio/kick_curve.cc
              src/yisibot_radio/crc.cc
              src/yisibot_radio/serial.cc
              src/radio/radio_helper.cc)
  TARGET_LINK_LIBRARIES(minutebot_radio_library constants_library net_library team_library amrl-shared-lib protobuf_library
  motion_library ${libs})

  ADD_EXECUTABLE(minutebot_radio_server
  src/radio/minutebot_radio_server_main.cc
  src/radio/minutebot_networkhandler.cc)
  TARGET_LINK_LIBRARIES(minutebot_radio_server minutebot_radio_library
  main_library
  constants_library net_library team_library amrl-shared-lib protobuf_library ${libs})
ENDIF()
