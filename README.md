# Simulation-based Causal Analysis and Reasoning System (SimCARS)
Provides a framework for carrying out causal discovery by utilising simulation to derive counterfactual data on the outcomes associated with different sets of autonomous agent decisions. As the name suggests, this framework was primarily designed with driving agents in mind, but is by no means limited to this.

# Compilation
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

