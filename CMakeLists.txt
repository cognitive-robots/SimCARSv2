cmake_minimum_required(VERSION 3.14)

project(simcars VERSION 0.0.1 LANGUAGES CXX)

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

add_subdirectory(extern/intelligent-driver-model)
add_subdirectory(extern/magic_enum)
add_subdirectory(extern/lz4_stream)
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
  include/ori/simcars/temporal/temporal_search_dictionary_abstract.hpp
  include/ori/simcars/temporal/proximal_temporal_dictionary.hpp
  include/ori/simcars/temporal/precedence_temporal_dictionary.hpp
  include/ori/simcars/temporal/temporal_rounding_dictionary.hpp
  include/ori/simcars/temporal/stateful_interface.hpp
  include/ori/simcars/temporal/living_interface.hpp
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
  src/map/lyft/lyft_map.cpp
  src/map/lyft/lyft_lane.cpp
  src/map/lyft/lyft_traffic_light.cpp
  src/map/highd/highd_lane.cpp
  src/map/highd/highd_map.cpp
  include/ori/simcars/map/soul_interface.hpp
  include/ori/simcars/map/soul_abstract.hpp
  include/ori/simcars/map/declarations.hpp
  include/ori/simcars/map/map_interface.hpp
  include/ori/simcars/map/file_based_map_interface.hpp
  include/ori/simcars/map/map_object_interface.hpp
  include/ori/simcars/map/file_based_map_abstract.hpp
  include/ori/simcars/map/map_object.hpp
  include/ori/simcars/map/lane_interface.hpp
  include/ori/simcars/map/lane_array_interface.hpp
  include/ori/simcars/map/lane_abstract.hpp
  include/ori/simcars/map/lane_array_abstract.hpp
  include/ori/simcars/map/living_lane_abstract.hpp
  include/ori/simcars/map/living_lane_stack_array.hpp
  include/ori/simcars/map/ghost_lane.hpp
  include/ori/simcars/map/ghost_lane_array.hpp
  include/ori/simcars/map/traffic_light_state_holder_interface.hpp
  include/ori/simcars/map/traffic_light_interface.hpp
  include/ori/simcars/map/traffic_light_array_interface.hpp
  include/ori/simcars/map/traffic_light_abstract.hpp
  include/ori/simcars/map/traffic_light_array_abstract.hpp
  include/ori/simcars/map/living_traffic_light_abstract.hpp
  include/ori/simcars/map/living_traffic_light_stack_array.hpp
  include/ori/simcars/map/ghost_traffic_light.hpp
  include/ori/simcars/map/ghost_traffic_light_array.hpp
  include/ori/simcars/map/map_grid_rect.hpp
  include/ori/simcars/map/lyft/lyft_declarations.hpp
  include/ori/simcars/map/lyft/lyft_map.hpp
  include/ori/simcars/map/lyft/lyft_lane.hpp
  include/ori/simcars/map/lyft/lyft_traffic_light.hpp
  include/ori/simcars/map/highd/highd_declarations.hpp
  include/ori/simcars/map/highd/highd_map.hpp
  include/ori/simcars/map/highd/highd_lane.hpp
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
  lz4_stream
  rapidcsv
)
add_dependencies(simcars_map simcars_utils simcars_structures simcars_geometry simcars_temporal)
target_compile_options(simcars_map PRIVATE -fPIC)
set_target_properties(simcars_map PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_agent STATIC
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
  src/agent/basic_driving_agent_state.cpp
  src/agent/view_read_only_driving_agent_state.cpp
  src/agent/view_driving_agent_state.cpp
  src/agent/basic_read_only_driving_scene_state.cpp
  src/agent/basic_driving_scene_state.cpp
  src/agent/view_driving_scene_state.cpp
  src/agent/basic_goal_driving_agent_state.cpp
  src/agent/safe_speedy_driving_agent_reward_calculator.cpp
  src/agent/basic_fp_action_sampler.cpp
  src/agent/basic_driving_simulator.cpp
  src/agent/driving_simulation_agent.cpp
  src/agent/driving_simulation_scene.cpp
  src/agent/lyft/lyft_driving_agent.cpp
  src/agent/lyft/lyft_scene.cpp
  src/agent/highd/highd_driving_agent.cpp
  src/agent/highd/highd_scene.cpp
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
  include/ori/simcars/agent/action_sampler_interface.hpp
  include/ori/simcars/agent/simulator_interface.hpp
  include/ori/simcars/agent/simulation_scene_interface.hpp
  include/ori/simcars/agent/driving_simulation_agent_interface.hpp
  include/ori/simcars/agent/driving_simulation_scene_interface.hpp
  include/ori/simcars/agent/driving_agent_controller_interface.hpp
  include/ori/simcars/agent/driving_agent_reward_calculator_interface.hpp
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
  include/ori/simcars/agent/basic_driving_agent_controller.hpp
  include/ori/simcars/agent/safe_speedy_driving_agent_reward_calculator.hpp
  include/ori/simcars/agent/basic_fp_action_sampler.hpp
  include/ori/simcars/agent/lyft/lyft_driving_agent.hpp
  include/ori/simcars/agent/lyft/lyft_scene.hpp
  include/ori/simcars/agent/highd/highd_driving_agent.hpp
  include/ori/simcars/agent/highd/highd_scene.hpp
  include/ori/simcars/agent/csv/csv_scene.hpp
)
target_include_directories(simcars_agent
PUBLIC
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${RAPIDJSON_INCLUDE_DIRS}
)
target_link_libraries(simcars_agent
PUBLIC
  simcars_utils
  simcars_structures
  simcars_geometry
  simcars_temporal
  simcars_map
  magic_enum::magic_enum
  rapidcsv
  idm
PRIVATE
  lz4_stream
)
add_dependencies(simcars_agent simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
target_compile_options(simcars_agent PRIVATE -fPIC)
set_target_properties(simcars_agent PROPERTIES VERSION ${PROJECT_VERSION})

add_library(simcars_visualisation STATIC
  src/visualisation/utils.cpp
  src/visualisation/qsfml_canvas_abstract.cpp
  src/visualisation/qscene_widget.cpp
  include/ori/simcars/visualisation/utils.hpp
  include/ori/simcars/visualisation/qsfml_canvas_abstract.hpp
  include/ori/simcars/visualisation/qscene_widget.hpp
  include/ori/simcars/visualisation/qmap_scene_widget.hpp
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
  simcars_agent
  Qt5::Widgets
PRIVATE
  sfml-graphics
)
add_dependencies(simcars_visualisation simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
target_compile_options(simcars_visualisation PRIVATE -fPIC)
set_target_properties(simcars_visualisation PROPERTIES VERSION ${PROJECT_VERSION})


add_executable(trig_buff_test src/trig_buff_test/trig_buff_test.cpp)
target_link_libraries(trig_buff_test simcars_utils simcars_structures simcars_geometry)
add_dependencies(trig_buff_test simcars_utils simcars_structures simcars_geometry)


add_executable(lyft_map_test src/lyft_map_test/lyft_map_test.cpp)
target_link_libraries(lyft_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(lyft_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)

add_executable(lyft_scene_test src/lyft_scene_test/lyft_scene_test.cpp)
target_link_libraries(lyft_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(lyft_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(lyft_scene_action_extraction src/lyft_scene_action_extraction/lyft_scene_action_extraction.cpp)
target_link_libraries(lyft_scene_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(lyft_scene_action_extraction simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(lyft_qscene_widget_test src/lyft_qscene_widget_test/lyft_qscene_widget_test.cpp)
target_link_libraries(lyft_qscene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)
add_dependencies(lyft_qscene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)

add_executable(lyft_qmap_scene_widget_test src/lyft_qmap_scene_widget_test/lyft_qmap_scene_widget_test.cpp)
target_link_libraries(lyft_qmap_scene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)
add_dependencies(lyft_qmap_scene_widget_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)

add_executable(lyft_simcars_demo src/lyft_simcars_demo/lyft_simcars_demo.cpp)
target_link_libraries(lyft_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)
add_dependencies(lyft_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)


add_executable(highd_map_test src/highd_map_test/highd_map_test.cpp)
target_link_libraries(highd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)
add_dependencies(highd_map_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map)

add_executable(highd_scene_test src/highd_scene_test/highd_scene_test.cpp)
target_link_libraries(highd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(highd_scene_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(highd_simulation_test src/highd_simulation_test/highd_simulation_test.cpp)
target_link_libraries(highd_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(highd_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(highd_bulk_simulation_test src/highd_bulk_simulation_test/highd_bulk_simulation_test.cpp)
target_link_libraries(highd_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(highd_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(highd_json_meta_simulation_test src/highd_json_meta_simulation_test/highd_json_meta_simulation_test.cpp)
target_link_libraries(highd_json_meta_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(highd_json_meta_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(highd_json_meta_bulk_simulation_test src/highd_json_meta_bulk_simulation_test/highd_json_meta_bulk_simulation_test.cpp)
target_link_libraries(highd_json_meta_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)
add_dependencies(highd_json_meta_bulk_simulation_test simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent)

add_executable(highd_simcars_demo src/highd_simcars_demo/highd_simcars_demo.cpp)
target_link_libraries(highd_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)
add_dependencies(highd_simcars_demo simcars_utils simcars_structures simcars_geometry simcars_temporal simcars_map simcars_agent simcars_visualisation)
