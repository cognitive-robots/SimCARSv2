cmake_minimum_required(VERSION 3.14)

project(simcars VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -flto")

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Eigen3 REQUIRED)
find_package(RapidJSON REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(SFML REQUIRED COMPONENTS graphics)
find_package(lanelet2_core REQUIRED)
find_package(lanelet2_io REQUIRED)
find_package(lanelet2_projection REQUIRED)
find_package(lanelet2_traffic_rules REQUIRED)
find_package(lanelet2_routing REQUIRED)

add_subdirectory(extern/magic_enum)
if (NOT TARGET rapidcsv)
  add_subdirectory(extern/rapidcsv)
endif()

add_library(simcars_utils STATIC
  src/utils/sanity_check.cpp
  include/ori/simcars/utils/exceptions.hpp
)
target_include_directories(simcars_utils
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(simcars_utils PRIVATE -fPIC)
set_target_properties(simcars_utils PROPERTIES LINKER_LANGUAGE CXX)
set_target_properties(simcars_utils PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_structures STATIC
  src/structures/sanity_check.cpp
  include/ori/simcars/structures/container_interface.hpp
  include/ori/simcars/structures/array_interface.hpp
  include/ori/simcars/structures/list_interface.hpp
  include/ori/simcars/structures/ra_mod_list_interface.hpp
  include/ori/simcars/structures/stack_interface.hpp
  include/ori/simcars/structures/stack_array_interface.hpp
  include/ori/simcars/structures/queue_interface.hpp
  include/ori/simcars/structures/queue_array_interface.hpp
  include/ori/simcars/structures/deque_interface.hpp
  include/ori/simcars/structures/deque_array_interface.hpp
  include/ori/simcars/structures/set_interface.hpp
  include/ori/simcars/structures/dictionary_interface.hpp
  include/ori/simcars/structures/stl/stl_stack_array.hpp
  include/ori/simcars/structures/stl/stl_deque_array.hpp
  include/ori/simcars/structures/stl/stl_set.hpp
  include/ori/simcars/structures/stl/stl_ordered_set.hpp
  include/ori/simcars/structures/stl/stl_dictionary.hpp
  include/ori/simcars/structures/stl/stl_ordered_dictionary.hpp
  include/ori/simcars/structures/stl/stl_concat_array.hpp
)
target_include_directories(simcars_structures
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(simcars_structures
PUBLIC
  simcars_utils
)
add_dependencies(simcars_structures simcars_utils)
target_compile_options(simcars_structures PRIVATE -fPIC)
set_target_properties(simcars_structures PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_geometry STATIC
  src/geometry/trig_buff.cpp
  src/geometry/tri.cpp
  src/geometry/rect.cpp
  src/geometry/o_rect.cpp
  include/ori/simcars/geometry/defines.hpp
  include/ori/simcars/geometry/typedefs.hpp
  include/ori/simcars/geometry/enums.hpp
  include/ori/simcars/geometry/trig_buff.hpp
  include/ori/simcars/geometry/tri.hpp
  include/ori/simcars/geometry/rect.hpp
  include/ori/simcars/geometry/grid_rect.hpp
  include/ori/simcars/geometry/o_rect.hpp
  include/ori/simcars/geometry/grid_dictionary.hpp
)
target_include_directories(simcars_geometry
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(simcars_geometry
PUBLIC
  simcars_utils
  simcars_structures
  Eigen3::Eigen
PRIVATE
  magic_enum::magic_enum
)
add_dependencies(simcars_geometry simcars_utils simcars_structures)
target_compile_options(simcars_geometry PRIVATE -fPIC)
set_target_properties(simcars_geometry PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_temporal STATIC
  src/temporal/sanity_check.cpp
  include/ori/simcars/temporal/typedefs.hpp
  include/ori/simcars/temporal/temporal_dictionary.hpp
)
target_include_directories(simcars_temporal
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(simcars_temporal
PUBLIC
  simcars_utils
  simcars_structures
)
add_dependencies(simcars_temporal simcars_utils simcars_structures)
target_compile_options(simcars_temporal PRIVATE -fPIC)
set_target_properties(simcars_temporal PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_map STATIC
  src/map/driving_map_object_abstract.cpp
  src/map/lane_abstract.cpp
  src/map/lane_branch.cpp
  src/map/highd/highd_lane.cpp
  src/map/highd/highd_map.cpp
  src/map/laneletd/laneletd_lane.cpp
  src/map/laneletd/laneletd_map.cpp
  src/map/plg/plg_lane.cpp
  src/map/plg/plg_map.cpp
  src/map/thor_magni/thor_magni_map.cpp
  src/map/thor_magni/thor_magni_node.cpp
  include/ori/simcars/map/declarations.hpp
  include/ori/simcars/map/map_interface.hpp
  include/ori/simcars/map/driving_map_interface.hpp
  include/ori/simcars/map/driving_map_object_interface.hpp
  include/ori/simcars/map/lane_interface.hpp
  include/ori/simcars/map/ped_map_interface.hpp
  include/ori/simcars/map/ped_map_object_interface.hpp
  include/ori/simcars/map/node_interface.hpp
  include/ori/simcars/map/driving_map_object_abstract.hpp
  include/ori/simcars/map/lane_abstract.hpp
  include/ori/simcars/map/lane_branch.hpp
  include/ori/simcars/map/highd/highd_declarations.hpp
  include/ori/simcars/map/highd/highd_map.hpp
  include/ori/simcars/map/highd/highd_lane.hpp
  include/ori/simcars/map/laneletd/laneletd_declarations.hpp
  include/ori/simcars/map/laneletd/laneletd_map.hpp
  include/ori/simcars/map/laneletd/laneletd_lane.hpp
  include/ori/simcars/map/plg/plg_declarations.hpp
  include/ori/simcars/map/plg/plg_map.hpp
  include/ori/simcars/map/plg/plg_lane.hpp
  include/ori/simcars/map/thor_magni/thor_magni_map.hpp
  include/ori/simcars/map/thor_magni/thor_magni_node.hpp
)
target_include_directories(simcars_map
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
  ${RAPIDJSON_INCLUDE_DIRS}
)
target_link_libraries(simcars_map
PUBLIC
  simcars_utils
  simcars_structures
  simcars_geometry
  simcars_temporal
  ${lanelet2_core_LIBRARIES}
  ${lanelet2_io_LIBRARIES}
  ${lanelet2_projection_LIBRARIES}
  ${lanelet2_traffic_rules_LIBRARIES}
  ${lanelet2_routing_LIBRARIES}
  sfml-graphics
PRIVATE
#  lz4_stream
  rapidcsv
)
add_dependencies(simcars_map simcars_utils simcars_structures simcars_geometry simcars_temporal)
target_compile_options(simcars_map PRIVATE -fPIC)
set_target_properties(simcars_map PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_causal STATIC
  src/causal/variable_context.cpp
  src/causal/variable_types/exogenous/id_fixed.cpp
  src/causal/variable_types/exogenous/id_socket.cpp
  src/causal/variable_types/exogenous/scalar_fixed.cpp
  src/causal/variable_types/exogenous/scalar_socket.cpp
  src/causal/variable_types/exogenous/time_fixed.cpp
  src/causal/variable_types/exogenous/time_socket.cpp
  src/causal/variable_types/exogenous/duration_fixed.cpp
  src/causal/variable_types/exogenous/vector_fixed.cpp
  src/causal/variable_types/exogenous/vector_socket.cpp
  src/causal/variable_types/endogenous/id_proxy.cpp
  src/causal/variable_types/endogenous/ids_previous_time_step.cpp
  src/causal/variable_types/endogenous/scalar_proxy.cpp
  src/causal/variable_types/endogenous/scalar_proxy.cpp
  src/causal/variable_types/endogenous/scalar_absolute.cpp
  src/causal/variable_types/endogenous/scalar_negation.cpp
  src/causal/variable_types/endogenous/scalar_reciprocal.cpp
  src/causal/variable_types/endogenous/scalar_exponent.cpp
  src/causal/variable_types/endogenous/scalar_max.cpp
  src/causal/variable_types/endogenous/scalar_min.cpp
  src/causal/variable_types/endogenous/scalar_sum.cpp
  src/causal/variable_types/endogenous/scalar_binary_mean.cpp
  src/causal/variable_types/endogenous/scalar_product.cpp
  src/causal/variable_types/endogenous/scalar_time_step_size_product.cpp
  src/causal/variable_types/endogenous/scalar_time_step_size_quotient.cpp
  src/causal/variable_types/endogenous/scalar_conditional.cpp
  src/causal/variable_types/endogenous/scalar_time_conditional.cpp
  src/causal/variable_types/endogenous/scalar_previous_time_step.cpp
  src/causal/variable_types/endogenous/scalar_buffer.cpp
  src/causal/variable_types/endogenous/scalar_set_min.cpp
  src/causal/variable_types/endogenous/scalar_set_sum.cpp
  src/causal/variable_types/endogenous/scalar_range_calc.cpp
  src/causal/variable_types/endogenous/time_current_time_difference.cpp
  src/causal/variable_types/endogenous/time_range_calc.cpp
  src/causal/variable_types/endogenous/duration_seconds_cast.cpp
  src/causal/variable_types/endogenous/vector_angle_construction.cpp
  src/causal/variable_types/endogenous/vector_xy_construction.cpp
  src/causal/variable_types/endogenous/vector_proxy.cpp
  src/causal/variable_types/endogenous/vector_x.cpp
  src/causal/variable_types/endogenous/vector_y.cpp
  src/causal/variable_types/endogenous/vector_negation.cpp
  src/causal/variable_types/endogenous/vector_norm.cpp
  src/causal/variable_types/endogenous/vector_normalisation.cpp
  src/causal/variable_types/endogenous/vector_sum.cpp
  src/causal/variable_types/endogenous/vector_binary_mean.cpp
  src/causal/variable_types/endogenous/vector_scalar_product.cpp
  src/causal/variable_types/endogenous/vector_time_step_size_product.cpp
  src/causal/variable_types/endogenous/vector_time_step_size_quotient.cpp
  src/causal/variable_types/endogenous/vector_dot_product.cpp
  src/causal/variable_types/endogenous/vector_cross_product.cpp
  src/causal/variable_types/endogenous/vector_conditional.cpp
  src/causal/variable_types/endogenous/vector_time_conditional.cpp
  src/causal/variable_types/endogenous/vector_previous_time_step.cpp
  src/causal/variable_types/endogenous/vector_buffer.cpp
  src/causal/variable_types/endogenous/vector_pair_first.cpp
  src/causal/variable_types/endogenous/vector_pair_second.cpp
  src/causal/variable_types/endogenous/vector_set_sum.cpp
  src/causal/variable_types/endogenous/matrix_angle_construction.cpp
  src/causal/variable_types/endogenous/matrix_vector_product.cpp
  src/causal/variable_types/endogenous/o_rect_construction.cpp
  src/causal/variable_types/endogenous/o_rect_collision.cpp
  src/causal/variable_types/endogenous/o_rect_contact.cpp
  src/causal/variable_types/endogenous/o_rect_dist_headway.cpp
  src/causal/variable_types/endogenous/lane_encapsulation.cpp
  src/causal/variable_types/endogenous/lane_encapsulating.cpp
  src/causal/variable_types/endogenous/lane_selectable.cpp
  src/causal/variable_types/endogenous/lane_map_point.cpp
  src/causal/variable_types/endogenous/lane_transitions_calc.cpp
  src/causal/variable_types/endogenous/node_centroid.cpp
  src/causal/variable_types/endogenous/node_closest.cpp
  src/causal/variable_types/endogenous/node_adjacent.cpp
  include/ori/simcars/causal/variable_interface.hpp
  include/ori/simcars/causal/exogenous_variable_interface.hpp
  include/ori/simcars/causal/endogenous_variable_interface.hpp
  include/ori/simcars/causal/unary_endogenous_variable_abstract.hpp
  include/ori/simcars/causal/binary_endogenous_variable_abstract.hpp
  include/ori/simcars/causal/ternary_endogenous_variable_abstract.hpp
  include/ori/simcars/causal/quaternary_endogenous_variable_abstract.hpp
  include/ori/simcars/causal/set_endogenous_variable_abstract.hpp
  include/ori/simcars/causal/variable_context.hpp
  include/ori/simcars/causal/variable_types/exogenous/id_fixed.hpp
  include/ori/simcars/causal/variable_types/exogenous/id_socket.hpp
  include/ori/simcars/causal/variable_types/exogenous/scalar_fixed.hpp
  include/ori/simcars/causal/variable_types/exogenous/scalar_socket.hpp
  include/ori/simcars/causal/variable_types/exogenous/time_fixed.hpp
  include/ori/simcars/causal/variable_types/exogenous/time_socket.hpp
  include/ori/simcars/causal/variable_types/exogenous/duration_fixed.hpp
  include/ori/simcars/causal/variable_types/exogenous/vector_fixed.hpp
  include/ori/simcars/causal/variable_types/exogenous/vector_socket.hpp
  include/ori/simcars/causal/variable_types/endogenous/id_proxy.hpp
  include/ori/simcars/causal/variable_types/endogenous/ids_previous_time_step.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_proxy.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_absolute.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_exponent.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_max.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_min.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_sum.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_binary_mean.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_time_step_size_quotient.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_conditional.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_time_conditional.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_previous_time_step.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_buffer.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_set_min.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_set_sum.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_range_calc.hpp
  include/ori/simcars/causal/variable_types/endogenous/time_current_time_difference.hpp
  include/ori/simcars/causal/variable_types/endogenous/time_range_calc.hpp
  include/ori/simcars/causal/variable_types/endogenous/duration_seconds_cast.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_angle_construction.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_xy_construction.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_proxy.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_x.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_y.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_negation.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_norm.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_normalisation.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_sum.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_binary_mean.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_scalar_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_time_step_size_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_time_step_size_quotient.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_dot_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_cross_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_conditional.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_time_conditional.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_previous_time_step.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_buffer.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_pair_first.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_pair_second.hpp
  include/ori/simcars/causal/variable_types/endogenous/vector_set_sum.hpp
  include/ori/simcars/causal/variable_types/endogenous/matrix_angle_construction.hpp
  include/ori/simcars/causal/variable_types/endogenous/matrix_vector_product.hpp
  include/ori/simcars/causal/variable_types/endogenous/o_rect_construction.hpp
  include/ori/simcars/causal/variable_types/endogenous/o_rect_collision.hpp
  include/ori/simcars/causal/variable_types/endogenous/o_rect_contact.hpp
  include/ori/simcars/causal/variable_types/endogenous/o_rect_dist_headway.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_encapsulation.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_encapsulating.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_selectable.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_transitions_calc.hpp
  include/ori/simcars/causal/variable_types/endogenous/node_centroid.hpp
  include/ori/simcars/causal/variable_types/endogenous/node_closest.hpp
  include/ori/simcars/causal/variable_types/endogenous/node_adjacent.hpp
)
target_include_directories(simcars_causal
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(simcars_causal
PUBLIC
  simcars_utils
  simcars_structures
  simcars_geometry
  simcars_temporal
  simcars_map
)
add_dependencies(simcars_causal simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
target_compile_options(simcars_causal PRIVATE -fPIC)
set_target_properties(simcars_causal PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_agents STATIC
  src/agents/point_mass.cpp
  src/agents/point_mass_sim.cpp
  src/agents/point_mass_env.cpp
  src/agents/rect_rigid_body.cpp
  src/agents/rect_rigid_body_sim.cpp
  src/agents/rect_rigid_body_env.cpp
  src/agents/fwd_car.cpp
  src/agents/fwd_car_sim.cpp
  src/agents/ped.cpp
  src/agents/ped_sim.cpp
  src/agents/control_fwd_car.cpp
  src/agents/motor_torque_control_fwd_car.cpp
  src/agents/steer_control_fwd_car.cpp
  src/agents/full_control_fwd_car.cpp
  src/agents/full_control_fwd_car_sim.cpp
  src/agents/control_ped.cpp
  src/agents/goal_force_control_ped.cpp
  src/agents/plan_fwd_car.cpp
  src/agents/greedy_plan_fwd_car.cpp
  src/agents/plan_ped.cpp
  src/agents/greedy_plan_ped.cpp
  src/agents/action_intervention_fwd_car.cpp
  src/agents/default_fwd_car_outcome_sim.cpp
  src/agents/default_fwd_car_reward_calc.cpp
  src/agents/fwd_car_action_extractor.cpp
  src/agents/ped_action_extractor.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_action_fixed.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_action_socket.cpp
  src/agents/causal/variable_types/exogenous/ped_outcome_parameters_fixed.cpp
  src/agents/causal/variable_types/exogenous/ped_reward_parameters_fixed.cpp
  src/agents/causal/variable_types/exogenous/ped_action_socket.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_reward_parameters_proxy.cpp
  src/agents/causal/variable_types/endogenous/ped_reward_parameters_proxy.cpp
  src/agents/causal/variable_types/endogenous/id_goal_val_part.cpp
  src/agents/causal/variable_types/endogenous/id_goal_time_part.cpp
  src/agents/causal/variable_types/endogenous/scalar_goal_val_part.cpp
  src/agents/causal/variable_types/endogenous/scalar_goal_time_part.cpp
  src/agents/causal/variable_types/endogenous/generate_fwd_car_actions.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_action_buffer.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pair_action_part.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pairs_buffer.cpp
  src/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.cpp
  src/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.cpp
  src/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.cpp
  src/agents/causal/variable_types/endogenous/generate_ped_actions.cpp
  src/agents/causal/variable_types/endogenous/ped_action_buffer.cpp
  src/agents/causal/variable_types/endogenous/ped_action_node_part.cpp
  src/agents/causal/variable_types/endogenous/ped_outcome_action_pair_action_part.cpp
  src/agents/causal/variable_types/endogenous/ped_outcome_action_pairs_buffer.cpp
  src/agents/causal/variable_types/endogenous/max_reward_ped_action.cpp
  src/agents/causal/variable_types/endogenous/calc_ped_action_outcome_reward.cpp
  src/agents/causal/variable_types/endogenous/sim_ped_action_outcome.cpp
  src/agents/highd/highd_fwd_car_scene.cpp
  src/agents/otherd/otherd_fwd_car_scene.cpp
  src/agents/thor_magni/thor_magni_ped_scene.cpp
  include/ori/simcars/agents/declarations.hpp
  include/ori/simcars/agents/typedefs.hpp
  include/ori/simcars/agents/fwd_car_outcome_sim_interface.hpp
  include/ori/simcars/agents/fwd_car_reward_calc_interface.hpp
  include/ori/simcars/agents/fwd_car_scene_interface.hpp
  include/ori/simcars/agents/ped_outcome_sim_interface.hpp
  include/ori/simcars/agents/ped_reward_calc_interface.hpp
  include/ori/simcars/agents/ped_scene_interface.hpp
  include/ori/simcars/agents/goal.hpp
  include/ori/simcars/agents/point_mass.hpp
  include/ori/simcars/agents/point_mass_sim.hpp
  include/ori/simcars/agents/point_mass_env.hpp
  include/ori/simcars/agents/rect_rigid_body.hpp
  include/ori/simcars/agents/rect_rigid_body_sim.hpp
  include/ori/simcars/agents/rect_rigid_body_env.hpp
  include/ori/simcars/agents/fwd_car.hpp
  include/ori/simcars/agents/fwd_car_sim.hpp
  include/ori/simcars/agents/fwd_car_action.hpp
  include/ori/simcars/agents/fwd_car_outcome.hpp
  include/ori/simcars/agents/fwd_car_rewards.hpp
  include/ori/simcars/agents/fwd_car_sim_parameters.hpp
  include/ori/simcars/agents/fwd_car_reward_parameters.hpp
  include/ori/simcars/agents/ped.hpp
  include/ori/simcars/agents/ped_sim.hpp
  include/ori/simcars/agents/ped_action.hpp
  include/ori/simcars/agents/ped_outcome.hpp
  include/ori/simcars/agents/ped_rewards.hpp
  include/ori/simcars/agents/ped_sim_parameters.hpp
  include/ori/simcars/agents/ped_reward_parameters.hpp
  include/ori/simcars/agents/control_fwd_car.hpp
  include/ori/simcars/agents/motor_torque_control_fwd_car.hpp
  include/ori/simcars/agents/steer_control_fwd_car.hpp
  include/ori/simcars/agents/full_control_fwd_car.hpp
  include/ori/simcars/agents/full_control_fwd_car_sim.hpp
  include/ori/simcars/agents/control_ped.hpp
  include/ori/simcars/agents/goal_force_control_ped.hpp
  include/ori/simcars/agents/plan_fwd_car.hpp
  include/ori/simcars/agents/greedy_plan_fwd_car.hpp
  include/ori/simcars/agents/action_intervention_fwd_car.hpp
  include/ori/simcars/agents/plan_ped.hpp
  include/ori/simcars/agents/greedy_plan_ped.hpp
  include/ori/simcars/agents/default_fwd_car_outcome_sim.hpp
  include/ori/simcars/agents/default_fwd_car_reward_calc.hpp
  include/ori/simcars/agents/fwd_car_action_extractor.hpp
  include/ori/simcars/agents/ped_action_extractor.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_action_socket.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/ped_outcome_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/ped_reward_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/ped_action_socket.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_reward_parameters_proxy.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/ped_reward_parameters_proxy.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_time_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_buffer.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pair_action_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_outcome_action_pairs_buffer.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/generate_ped_actions.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/ped_action_buffer.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/ped_action_node_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/ped_outcome_action_pair_action_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/ped_outcome_action_pairs_buffer.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/max_reward_ped_action.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/calc_ped_action_outcome_reward.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/sim_ped_action_outcome.hpp
  include/ori/simcars/agents/highd/highd_fwd_car_scene.hpp
  include/ori/simcars/agents/otherd/otherd_fwd_car_scene.hpp
  include/ori/simcars/agents/thor_magni/thor_magni_ped_scene.hpp
)
target_include_directories(simcars_agents
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${RAPIDJSON_INCLUDE_DIRS}
)
target_link_libraries(simcars_agents
PUBLIC
  simcars_utils
  simcars_structures
  simcars_geometry
  simcars_temporal
  simcars_map
  simcars_causal
  rapidcsv
)
add_dependencies(simcars_agents simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_causal)
target_compile_options(simcars_agents PRIVATE -fPIC)
set_target_properties(simcars_agents PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_visualisation STATIC
  src/visualisation/qsfml_canvas_abstract.cpp
  src/visualisation/qdriving_agents_widget.cpp
  src/visualisation/qdriving_map_agents_widget.cpp
  src/visualisation/qped_agents_widget.cpp
  src/visualisation/qped_map_agents_widget.cpp
  include/ori/simcars/visualisation/qsfml_canvas_abstract.hpp
  include/ori/simcars/visualisation/qdriving_agents_widget.hpp
  include/ori/simcars/visualisation/qdriving_map_agents_widget.hpp
  include/ori/simcars/visualisation/qped_agents_widget.hpp
  include/ori/simcars/visualisation/qped_map_agents_widget.hpp
)
target_include_directories(simcars_visualisation
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(simcars_visualisation
PUBLIC
  simcars_utils
  simcars_structures
  simcars_geometry
  simcars_temporal
  simcars_map
  simcars_causal
  simcars_agents
  Qt5::Widgets
PRIVATE
  sfml-graphics
)
add_dependencies(simcars_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_causal simcars_agents)
target_compile_options(simcars_visualisation PRIVATE -fPIC)
set_target_properties(simcars_visualisation PROPERTIES VERSION ${PROJECT_VERSION})


add_executable(trig_buff_test src/trig_buff_test/trig_buff_test.cpp)
target_link_libraries(trig_buff_test simcars_utils simcars_structures simcars_geometry)
add_dependencies(trig_buff_test simcars_utils simcars_structures simcars_geometry)


add_executable(highd_map_test src/highd_map_test/highd_map_test.cpp)
target_link_libraries(highd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(highd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)

add_executable(highd_scene_test src/highd_scene_test/highd_scene_test.cpp)
target_link_libraries(highd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_action_extraction src/highd_action_extraction/highd_action_extraction.cpp)
target_link_libraries(highd_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_visualisation src/highd_visualisation/highd_visualisation.cpp)
target_link_libraries(highd_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_intervention src/highd_intervention/highd_intervention.cpp)
target_link_libraries(highd_intervention simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_intervention simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_action_explanation src/highd_action_explanation/highd_action_explanation.cpp)
target_link_libraries(highd_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_json_meta_action_explanation src/highd_json_meta_action_explanation/highd_json_meta_action_explanation.cpp)
target_link_libraries(highd_json_meta_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_json_meta_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_sim_visualisation src/highd_sim_visualisation/highd_sim_visualisation.cpp)
target_link_libraries(highd_sim_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_sim_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_alt_visualisation src/highd_alt_visualisation/highd_alt_visualisation.cpp)
target_link_libraries(highd_alt_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_alt_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)


add_executable(otherd_scene_test src/otherd_scene_test/otherd_scene_test.cpp)
target_link_libraries(otherd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(otherd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)


add_executable(laneletd_map_test src/laneletd_map_test/laneletd_map_test.cpp)
target_link_libraries(laneletd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(laneletd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)

add_executable(laneletd_visualisation src/laneletd_visualisation/laneletd_visualisation.cpp)
target_link_libraries(laneletd_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_visualisation)
add_dependencies(laneletd_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_visualisation)

add_executable(laneletd_action_extraction src/laneletd_action_extraction/laneletd_action_extraction.cpp)
target_link_libraries(laneletd_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(laneletd_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(laneletd_intervention src/laneletd_intervention/laneletd_intervention.cpp)
target_link_libraries(laneletd_intervention simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(laneletd_intervention simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(laneletd_action_explanation src/laneletd_action_explanation/laneletd_action_explanation.cpp)
target_link_libraries(laneletd_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(laneletd_action_explanation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(laneletd_alt_visualisation src/laneletd_alt_visualisation/laneletd_alt_visualisation.cpp)
target_link_libraries(laneletd_alt_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(laneletd_alt_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(laneletd_sim_visualisation src/laneletd_sim_visualisation/laneletd_sim_visualisation.cpp)
target_link_libraries(laneletd_sim_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(laneletd_sim_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)


add_executable(plg_map_test src/plg_map_test/plg_map_test.cpp)
target_link_libraries(plg_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(plg_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)


add_executable(thor_magni_scene_test src/thor_magni_scene_test/thor_magni_scene_test.cpp)
target_link_libraries(thor_magni_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(thor_magni_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(thor_magni_action_extraction src/thor_magni_action_extraction/thor_magni_action_extraction.cpp)
target_link_libraries(thor_magni_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(thor_magni_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(thor_magni_visualisation src/thor_magni_visualisation/thor_magni_visualisation.cpp)
target_link_libraries(thor_magni_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(thor_magni_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
