# Simulation-based Causal Analysis and Reasoning System (SimCARS)
Provides a framework for carrying out causal discovery by utilising simulation to derive counterfactual data on the outcomes associated with different sets of autonomous agent decisions. As the name suggests, this framework was primarily designed with driving agents in mind, but is by no means limited to this.

## Compilation
While the system can be compiled and run on a variety of systems with a variety of libraries, for the purposes of reproduction, the framework was last compiled using the following set of software/libraries:
* Qt Creator 4.11.0
* Qt 5.12.8
* GCC 9.3.0
From there, the project can be imported as a CMake project and compiled using the Qt Creator interface.

Alternatively assuming Qt and GCC are installed, the framework can be built using the following shell commands (starting in the project root directory):
```
mkdir build
cd build
cmake ..
make -j8
```
It may be necessary to use a tool like ccmake or cmake-gui in order to properly configure cmake for your system. Note: ```-j8``` can be omitted, it just specifies the maximum number of jobs to run at once.

## Built Executables

### Trigonometry Buffer Test
Tests that the trigonometry buffer compiles and runs without segmentation fault, also outputs execution times for the trigonometry buffer vs standard trigonometry functions.

```
usage: trig_buff_test
```

### Lyft
Executables specific to the Lyft Level-5 Prediction Dataset.

#### Map Test
Tests that the Lyft map compiles and runs without segmentation fault.

```
usage: lyft_map_test map_file_path
```

Parameters:
* map_file_path: Specifies the file path of the map to load. Input format expected to be LZ4 compressed JSON map (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).

#### Scene Test
Tests that the Lyft scene compiles and runs without segmentation fault.

```
usage: lyft_scene_test scene_file_path
```

Parameters:
* scene_file_path: Specifies the file path of the scene to load. Input format expected to be LZ4 compressed JSON scene (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).

#### Scene Action Extraction
Converts a Lyft map and scene to a CSV file containing time series data for each variable.

```
usage: lyft_scene_action_extraction input_map_file_path input_scene_file_path output_scene_file_path
```

Parameters:
* input_map_file_path: Specifies the file path of the map to load. Input format expected to be LZ4 compressed JSON map (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).
* input_scene_file_path: Specifies the file path of the scene to load. Input format expected to be LZ4 compressed JSON scene (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).
* output_scene_file_path: Specifies the file path to output the CSV file to.

#### QSceneWidget Test
Tests that the QSceneWidget compiles, runs without segmentation fault and visualises correctly. QSceneWidget is designed to visualise the movement of agents during a scene.

```
usage: lyft_qscene_widget_test scene_file_path
```

Parameters:
* scene_file_path: Specifies the file path of the scene to load. Input format expected to be LZ4 compressed JSON scene (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).

#### QMapSceneWidget Test
Tests that the QMapSceneWidget compiles, runs without segmentation fault and visualises correctly. QMapSceneWidget is designed to visualise the same as QSceneWidget with the addition of a map.

```
usage: lyft_qmap_scene_widget_test map_file_path scene_file_path
```

Parameters:
* map_file_path: Specifies the file path of the map to load. Input format expected to be LZ4 compressed JSON map (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).
* scene_file_path: Specifies the file path of the scene to load. Input format expected to be LZ4 compressed JSON scene (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).

#### SimCARS Demo
Visualises two scenes, on the left the original scene, and on the right the original scene until the half way point and a simulation of the scene from that point onwards. Intended to allow for the comparison of the simulated scene against the original scene.

```
usage: lyft_simcars_demo map_file_path scene_file_path
```

Parameters:
* map_file_path: Specifies the file path of the map to load. Input format expected to be LZ4 compressed JSON map (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).
* scene_file_path: Specifies the file path of the scene to load. Input format expected to be LZ4 compressed JSON scene (See https://github.com/cognitive-robots/lyft_prediction_dataset_tools).
