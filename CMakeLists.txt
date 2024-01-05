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

add_subdirectory(extern/magic_enum)
#add_subdirectory(extern/lz4_stream)
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
  src/map/map_object_abstract.cpp
  src/map/lane_abstract.cpp
  src/map/map_grid_rect.cpp
  src/map/lane_branch.cpp
  src/map/highd/highd_lane.cpp
  src/map/highd/highd_map.cpp
  src/map/plg/plg_lane.cpp
  src/map/plg/plg_map.cpp
  include/ori/simcars/map/declarations.hpp
  include/ori/simcars/map/map_interface.hpp
  include/ori/simcars/map/map_object_interface.hpp
  include/ori/simcars/map/lane_interface.hpp
  include/ori/simcars/map/map_object_abstract.hpp
  include/ori/simcars/map/lane_abstract.hpp
  include/ori/simcars/map/map_grid_rect.hpp
  include/ori/simcars/map/lane_branch.hpp
  include/ori/simcars/map/highd/highd_declarations.hpp
  include/ori/simcars/map/highd/highd_map.hpp
  include/ori/simcars/map/highd/highd_lane.hpp
  include/ori/simcars/map/plg/plg_declarations.hpp
  include/ori/simcars/map/plg/plg_map.hpp
  include/ori/simcars/map/plg/plg_lane.hpp
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
  src/causal/variable_types/endogenous/scalar_negation.cpp
  src/causal/variable_types/endogenous/scalar_reciprocal.cpp
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
  src/causal/variable_types/endogenous/lane_encapsulation.cpp
  src/causal/variable_types/endogenous/lane_encapsulating.cpp
  src/causal/variable_types/endogenous/lane_selectable.cpp
  src/causal/variable_types/endogenous/lane_map_point.cpp
  src/causal/variable_types/endogenous/lane_transitions_calc.cpp
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
  include/ori/simcars/causal/variable_types/endogenous/scalar_negation.hpp
  include/ori/simcars/causal/variable_types/endogenous/scalar_reciprocal.hpp
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
  include/ori/simcars/causal/variable_types/endogenous/lane_encapsulation.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_encapsulating.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_selectable.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_map_point.hpp
  include/ori/simcars/causal/variable_types/endogenous/lane_transitions_calc.hpp
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
  src/agents/rect_rigid_body.cpp
  src/agents/rect_rigid_body_sim.cpp
  src/agents/rect_rigid_body_env.cpp
  src/agents/fwd_car.cpp
  src/agents/fwd_car_sim.cpp
  src/agents/control_fwd_car.cpp
  src/agents/motor_torque_control_fwd_car.cpp
  src/agents/steer_control_fwd_car.cpp
  src/agents/full_control_fwd_car.cpp
  src/agents/full_control_fwd_car_sim.cpp
  src/agents/plan_fwd_car.cpp
  src/agents/greedy_plan_fwd_car.cpp
  src/agents/action_intervention_fwd_car.cpp
  src/agents/default_fwd_car_outcome_sim.cpp
  src/agents/default_fwd_car_reward_calc.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.cpp
  src/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.cpp
  src/agents/causal/variable_types/endogenous/id_goal_val_part.cpp
  src/agents/causal/variable_types/endogenous/id_goal_time_part.cpp
  src/agents/causal/variable_types/endogenous/scalar_goal_val_part.cpp
  src/agents/causal/variable_types/endogenous/scalar_goal_time_part.cpp
  src/agents/causal/variable_types/endogenous/generate_fwd_car_actions.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.cpp
  src/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.cpp
  src/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.cpp
  src/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.cpp
  src/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.cpp
  src/agents/highd/highd_fwd_car_scene.cpp
  include/ori/simcars/agents/declarations.hpp
  include/ori/simcars/agents/typedefs.hpp
  include/ori/simcars/agents/fwd_car_outcome_sim_interface.hpp
  include/ori/simcars/agents/fwd_car_reward_calc_interface.hpp
  include/ori/simcars/agents/fwd_car_scene_interface.hpp
  include/ori/simcars/agents/goal.hpp
  include/ori/simcars/agents/rect_rigid_body.hpp
  include/ori/simcars/agents/rect_rigid_body_sim.hpp
  include/ori/simcars/agents/rect_rigid_body_env.hpp
  include/ori/simcars/agents/fwd_car.hpp
  include/ori/simcars/agents/fwd_car_sim.hpp
  include/ori/simcars/agents/fwd_car_action.hpp
  include/ori/simcars/agents/fwd_car_outcome.hpp
  include/ori/simcars/agents/fwd_car_sim_parameters.hpp
  include/ori/simcars/agents/fwd_car_reward_parameters.hpp
  include/ori/simcars/agents/control_fwd_car.hpp
  include/ori/simcars/agents/motor_torque_control_fwd_car.hpp
  include/ori/simcars/agents/steer_control_fwd_car.hpp
  include/ori/simcars/agents/full_control_fwd_car.hpp
  include/ori/simcars/agents/full_control_fwd_car_sim.hpp
  include/ori/simcars/agents/plan_fwd_car.hpp
  include/ori/simcars/agents/greedy_plan_fwd_car.hpp
  include/ori/simcars/agents/action_intervention_fwd_car.hpp
  include/ori/simcars/agents/default_fwd_car_outcome_sim.hpp
  include/ori/simcars/agents/default_fwd_car_reward_calc.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_outcome_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/exogenous/fwd_car_reward_parameters_fixed.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/id_goal_val_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/id_goal_time_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_val_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/scalar_goal_time_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/generate_fwd_car_actions.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_speed_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/fwd_car_action_lane_part.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/max_reward_fwd_car_action.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/calc_fwd_car_action_outcome_reward.hpp
  include/ori/simcars/agents/causal/variable_types/endogenous/sim_fwd_car_action_outcome.hpp
  include/ori/simcars/agents/highd/highd_fwd_car_scene.hpp
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

#[[
add_library(simcars_agents STATIC
  src/agent/valueless_constant_abstract.cpp
  src/agent/valueless_variable_abstract.cpp
  src/agent/driving_agent_abstract.cpp
  src/agent/read_only_driving_agent_state_abstract.cpp
  src/agent/driving_scene_abstract.cpp
  src/agent/read_only_driving_scene_state_abstract.cpp
  src/agent/driving_scene_state_abstract.cpp
  src/agent/driving_simulation_agent_abstract.cpp
  src/agent/driving_simulation_scene_abstract.cpp
  src/agent/driving_agent_reward_calculator_abstract.cpp
  src/agent/driving_agent_agency_calculator_abstract.cpp
  src/agent/basic_driving_agent_state.cpp
  src/agent/view_read_only_driving_agent_state.cpp
  src/agent/view_driving_agent_state.cpp
  src/agent/basic_read_only_driving_scene_state.cpp
  src/agent/basic_driving_scene_state.cpp
  src/agent/view_driving_scene_state.cpp
  src/agent/basic_goal_driving_agent_state.cpp
  src/agent/safe_speedy_driving_agent_reward_calculator.cpp
  src/agent/basic_driving_agent_agency_calculator.cpp
  src/agent/basic_fp_action_sampler.cpp
  src/agent/basic_driving_simulator.cpp
  src/agent/driving_simulation_agent.cpp
  src/agent/driving_simulation_scene.cpp
  src/agent/driving_simulation_scene_factory.cpp
  src/agent/lyft/lyft_driving_agent.cpp
  src/agent/lyft/lyft_scene.cpp
  src/agent/highd/highd_driving_agent.cpp
  src/agent/highd/highd_scene.cpp
  src/agent/plg/plg_driving_agent.cpp
  src/agent/plg/plg_scene.cpp
  src/agent/csv/csv_scene.cpp
  include/ori/simcars/agent/defines.hpp
  include/ori/simcars/agent/declarations.hpp
  include/ori/simcars/agent/driving_declarations.hpp
  include/ori/simcars/agent/valueless_constant_interface.hpp
  include/ori/simcars/agent/valueless_event_interface.hpp
  include/ori/simcars/agent/valueless_variable_interface.hpp
  include/ori/simcars/agent/simulated_valueless_variable_interface.hpp
  include/ori/simcars/agent/constant_interface.hpp
  include/ori/simcars/agent/event_interface.hpp
  include/ori/simcars/agent/variable_interface.hpp
  include/ori/simcars/agent/simulated_variable_interface.hpp
  include/ori/simcars/agent/entity_interface.hpp
  include/ori/simcars/agent/read_only_entity_state_interface.hpp
  include/ori/simcars/agent/entity_state_interface.hpp
  include/ori/simcars/agent/driving_enums.hpp
  include/ori/simcars/agent/driving_agent_interface.hpp
  include/ori/simcars/agent/read_only_driving_agent_state_interface.hpp
  include/ori/simcars/agent/driving_agent_state_interface.hpp
  include/ori/simcars/agent/scene_interface.hpp
  include/ori/simcars/agent/read_only_scene_state_interface.hpp
  include/ori/simcars/agent/scene_state_interface.hpp
  include/ori/simcars/agent/driving_scene_interface.hpp
  include/ori/simcars/agent/read_only_driving_scene_state_interface.hpp
  include/ori/simcars/agent/driving_scene_state_interface.hpp
  include/ori/simcars/agent/file_based_scene_interface.hpp
  include/ori/simcars/agent/two_file_based_scene_interface.hpp
  include/ori/simcars/agent/controller_interface.hpp
  include/ori/simcars/agent/reward_calculator_interface.hpp
  include/ori/simcars/agent/agency_calculator_interface.hpp
  include/ori/simcars/agent/action_sampler_interface.hpp
  include/ori/simcars/agent/simulator_interface.hpp
  include/ori/simcars/agent/simulation_scene_interface.hpp
  include/ori/simcars/agent/simulation_scene_factory_interface.hpp
  include/ori/simcars/agent/driving_simulation_agent_interface.hpp
  include/ori/simcars/agent/driving_simulation_scene_interface.hpp
  include/ori/simcars/agent/driving_agent_controller_interface.hpp
  include/ori/simcars/agent/driving_agent_reward_calculator_interface.hpp
  include/ori/simcars/agent/driving_agent_agency_calculator_interface.hpp
  include/ori/simcars/agent/driving_simulator_interface.hpp
  include/ori/simcars/agent/valueless_constant_abstract.hpp
  include/ori/simcars/agent/valueless_variable_abstract.hpp
  include/ori/simcars/agent/constant_abstract.hpp
  include/ori/simcars/agent/event_abstract.hpp
  include/ori/simcars/agent/variable_abstract.hpp
  include/ori/simcars/agent/simulated_variable_abstract.hpp
  include/ori/simcars/agent/driving_agent_abstract.hpp
  include/ori/simcars/agent/read_only_driving_agent_state_abstract.hpp
  include/ori/simcars/agent/driving_agent_state_abstract.hpp
  include/ori/simcars/agent/driving_scene_abstract.hpp
  include/ori/simcars/agent/read_only_driving_scene_state_abstract.hpp
  include/ori/simcars/agent/driving_scene_state_abstract.hpp
  include/ori/simcars/agent/file_based_scene_abstract.hpp
  include/ori/simcars/agent/two_file_based_scene_abstract.hpp
  include/ori/simcars/agent/driving_simulation_agent_abstract.hpp
  include/ori/simcars/agent/driving_simulation_scene_abstract.hpp
  include/ori/simcars/agent/driving_agent_reward_calculator_abstract.hpp
  include/ori/simcars/agent/driving_agent_agency_calculator_abstract.hpp
  include/ori/simcars/agent/goal.hpp
  include/ori/simcars/agent/basic_constant.hpp
  include/ori/simcars/agent/basic_event.hpp
  include/ori/simcars/agent/basic_variable.hpp
  include/ori/simcars/agent/basic_driving_agent_state.hpp
  include/ori/simcars/agent/view_read_only_driving_agent_state.hpp
  include/ori/simcars/agent/view_driving_agent_state.hpp
  include/ori/simcars/agent/basic_read_only_driving_scene_state.hpp
  include/ori/simcars/agent/basic_driving_scene_state.hpp
  include/ori/simcars/agent/view_driving_scene_state.hpp
  include/ori/simcars/agent/basic_goal_driving_agent_state.hpp
  include/ori/simcars/agent/driving_goal_extraction_agent.hpp
  include/ori/simcars/agent/driving_goal_extraction_scene.hpp
  include/ori/simcars/agent/basic_simulated_variable.hpp
  include/ori/simcars/agent/basic_driving_simulator.hpp
  include/ori/simcars/agent/driving_simulation_agent.hpp
  include/ori/simcars/agent/driving_simulation_scene.hpp
  include/ori/simcars/agent/driving_simulation_scene_factory.hpp
  include/ori/simcars/agent/basic_driving_agent_controller.hpp
  include/ori/simcars/agent/safe_speedy_driving_agent_reward_calculator.hpp
  include/ori/simcars/agent/basic_driving_agent_agency_calculator.hpp
  include/ori/simcars/agent/basic_fp_action_sampler.hpp
  include/ori/simcars/agent/lyft/lyft_driving_agent.hpp
  include/ori/simcars/agent/lyft/lyft_scene.hpp
  include/ori/simcars/agent/highd/highd_driving_agent.hpp
  include/ori/simcars/agent/highd/highd_scene.hpp
  include/ori/simcars/agent/plg/plg_driving_agent.hpp
  include/ori/simcars/agent/plg/plg_scene.hpp
  include/ori/simcars/agent/csv/csv_scene.hpp
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
PRIVATE
  magic_enum::magic_enum
  rapidcsv
#  lz4_stream
)
add_dependencies(simcars_agents simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
target_compile_options(simcars_agents PRIVATE -fPIC)
set_target_properties(simcars_agents PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_causal STATIC
  src/causal/necessary_fp_goal_causal_link_tester.cpp
  include/ori/simcars/causal/causal_link_tester_interface.hpp
  include/ori/simcars/causal/causal_discoverer_interface.hpp
  include/ori/simcars/causal/necessary_fp_goal_causal_link_tester.hpp
  include/ori/simcars/causal/necessary_driving_causal_discoverer.hpp
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
  simcars_agents
PRIVATE
  pthread
)
add_dependencies(simcars_causal simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
target_compile_options(simcars_causal PRIVATE -fPIC)
set_target_properties(simcars_causal PROPERTIES VERSION ${PROJECT_VERSION})
]]

add_library(simcars_visualisation STATIC
  src/visualisation/qsfml_canvas_abstract.cpp
  src/visualisation/qagents_widget.cpp
  src/visualisation/qmap_agents_widget.cpp
  include/ori/simcars/visualisation/qsfml_canvas_abstract.hpp
  include/ori/simcars/visualisation/qagents_widget.hpp
  include/ori/simcars/visualisation/qmap_agents_widget.hpp
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

#[[
add_executable(highd_simulation_test src/highd_simulation_test/highd_simulation_test.cpp)
target_link_libraries(highd_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_bulk_simulation_test src/highd_bulk_simulation_test/highd_bulk_simulation_test.cpp)
target_link_libraries(highd_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_json_meta_simulation_test src/highd_json_meta_simulation_test/highd_json_meta_simulation_test.cpp)
target_link_libraries(highd_json_meta_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_json_meta_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_json_meta_bulk_simulation_test src/highd_json_meta_bulk_simulation_test/highd_json_meta_bulk_simulation_test.cpp)
target_link_libraries(highd_json_meta_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(highd_json_meta_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(highd_json_meta_link_test src/highd_json_meta_link_test/highd_json_meta_link_test.cpp)
target_link_libraries(highd_json_meta_link_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_causal)
add_dependencies(highd_json_meta_link_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_causal)

add_executable(highd_json_meta_causal_discovery src/highd_json_meta_causal_discovery/highd_json_meta_causal_discovery.cpp)
target_link_libraries(highd_json_meta_causal_discovery simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_causal)
add_dependencies(highd_json_meta_causal_discovery simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_causal)

add_executable(highd_simcars_demo src/highd_simcars_demo/highd_simcars_demo.cpp)
target_link_libraries(highd_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_json_meta_simcars_intervention_demo src/highd_json_meta_simcars_intervention_demo/highd_json_meta_simcars_intervention_demo.cpp)
target_link_libraries(highd_json_meta_simcars_intervention_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_json_meta_simcars_intervention_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)

add_executable(highd_json_meta_simcars_causal_discovery_demo src/highd_json_meta_simcars_causal_discovery_demo/highd_json_meta_simcars_causal_discovery_demo.cpp)
target_link_libraries(highd_json_meta_simcars_causal_discovery_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(highd_json_meta_simcars_causal_discovery_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
]]


add_executable(plg_map_test src/plg_map_test/plg_map_test.cpp)
target_link_libraries(plg_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(plg_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)

#[[
add_executable(plg_scene_test src/plg_scene_test/plg_scene_test.cpp)
target_link_libraries(plg_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)
add_dependencies(plg_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents)

add_executable(plg_qmap_scene_widget_test src/plg_qmap_scene_widget_test/plg_qmap_scene_widget_test.cpp)
target_link_libraries(plg_qmap_scene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
add_dependencies(plg_qmap_scene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agents simcars_visualisation)
]]
