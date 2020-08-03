# Kinematics and Dynamics Model via Explicit Direct and Trigonometric Elimination of Kinematic Constraints

Authors:

Moritz Schappler, Torsten Lilge and Sami Haddadin

This repository provides supplementary material to the following submission to the IFToMM World Congress 2019:

```bib
@Conference{SchapplerLilHad2019,
  author    = {Schappler, Moritz and Lilge, Torsten and Haddadin, Sami},
  title     = {Kinematics and Dynamics Model via Explicit Direct and Trigonometric Elimination of Kinematic Constraints},
  booktitle = {Proc. of the 15th IFToMM World Congress},
  year      = {2019},
  month     = {October},
  doi       = {10.1007/978-3-030-20131-9_311},
}
```

## Abstract

The efficient implementation of kinematics and dynamics models is a key to model based control of mechatronic systems such as robots and wearable assistive devices.
This paper presents an approach for the derivation of these models in symbolic form for constrained systems based on the explicit elimination of the kinematic constraints using substitution variables with trigonometric expressions and the Lagrange equations of the second kind.
This represents an alternative solution to using the implicit form of the constraints or using the explicit elimination at comparable computational effort.
The method is applied to a novel exoskeleton designed for craftsmen force assistance, which consists of multiple planar closed kinematic loops and gear mechanisms.

## Contents

This repository contains Maple Worksheets and Matlab scripts to generate the exemplary mechanism ("exoskeleton") presented in the paper.
The implementation of the main method proposed in the paper is stored as a [Maple Toolbox for Hybrid Robot Dynamics](https://github.com/SchapplM/robsynth-modelgen).
Further robot examples such as robot palletizers are kept separately in the growing [Hybrid Robots Library](https://github.com/SchapplM/robsynth-serhybroblib).

* The directory `paper` contains the latex code for the manuscript (authors version).
* `exoskeleton_model` contains files for the generation and validation of the exoskeleton example.
  * The sub-directory `validation` contains the Matlab script `KAS5m7_robot_class_test.m` for a visualization of the mechanism (KAS stands for "Kraftassistenzsystem", "force assistance system"). The scripts `KAS5m5_KompStruktur_Test.m` and `KAS5m7_KompStruktur_Test.m` provide a kinematics verification of the symbolically generated model against a manually created version.
  * The directories `KAS5m3`, `KAS5m5`, ... contain Matlab functions and a test environment for different variants of the exoskeleton model. The variant KAS5m7 is the final stage which is used in the paper.
  * With `toolbox_testsuite.m` all automatically generated unit tests for the Matlab functions from the Maple toolbox are executed. With this the validity of the dynamics equations is proven.

### Dependencies

Running the unit tests (`toolbox_testsuite.m`) requires the following toolboxes:
 * [Matlab toolbox](https://github.com/SchapplM/matlab_toolbox) (tested with rev. 59c70af , 2020-05-25)
 * [Various external toolboxes](https://github.com/SchapplM/robotics-dep-ext) (tested with rev. 5a5fa3a, 2020-05-25)
 * [Robotics toolbox](https://github.com/SchapplM/robotics-toolbox) (tested with rev. e3783ce, 2020-07-01)

Running the visualisation test (`KAS5m7_robot_class_test.m`) further requires
* [Hybrid Robots Library](https://github.com/SchapplM/robsynth-serhybroblib). (tested with rev. 8e4d39cf , 2020-07-01)

To generate the model with Maple you need the 
* [Maple Toolbox for Hybrid Robot Dynamics](https://github.com/SchapplM/robsynth-modelgen). (tested with rev. 8945acd, 2020-06-30)

All Matlab toolboxes have to be added to the Matlab path by executing their corresponding `..._path_init.m` files.

### Generation of the Exoskeleton Model

* To regenerate the Matlab functions inside the sub-directory `matlabfcn` of each model, the files in the directory `model_generation` can be used. The following steps are necessary:
   * Download the [Maple dynamics toolbox](https://github.com/SchapplM/robsynth-modelgen) from GitHub. Set it up according to it's README file.
   * The code generation runs better on Linux operating system. For Windows an appropriate Bash terminal is necessary, see [toolbox Readme](https://github.com/SchapplM/robsynth-modelgen#unter-windows-windows-linux-subsystem).
   * The link to the toolbox in the file `hybrdyn_repo_path` has to be created (see template file with the same name)
   * Run `generate_models.sh` in a Bash terminal to create the second implementation of the model.

The code generation will run very long (multiple days on a desktop computer), since many different model variants and all kinds of terms of the dynamics equation are created.
It is therefore advised to only select one model for generation and comment out the unneeded models accordingly in the file `KAS_codegen_all_models.sh`.

### Evaluation of the Results

The computational effort reported in Tab.I of the paper was summed up from the Matlab files in `paper/proof`.
The full exoskeleton model was generated with a lower symbolic optimization routine of Maple (see `robot_env`-files), leading to higher number of computations (about 50%).
Therefore, a model of this complexity (number of joints and kinematic constraints) can be regarded as an upper limit for the feasibility of symbolic calculation of the multibody dynamics (with this approach and without simplifications).
