Ignition Gazebo Note
===

> Reference: [Ignition Gazebo Tutorial](https://gazebosim.org/api/sim/8/tutorials.html)
> 
> Dknt 2025.1

# 1 Introduction

## 1.1 Terminology

In Ignition Gazebo, a **World** is a complete description of a simulation, including all robots, movable and static objects, plugins, scenes and GUIs.

Every objects in the world, such as models, links, collisions, visuals, lights, joints, are called **Entity**. An entity is just a numeric ID assigned at runtime and may have several components attached to it.

**Component** can add functionality to an entity, such as pose and inertial. Users can create components by inheriting from the `BaseComponent` class or instantiating a template of `Component`.

**System** is logic that operates on all entities that have a given set of components. They are plugins that are loaded at runtime, and can be customized.

**Entity-component manager(ECM)**

**Level** is a part of the world, defined by a box volume and the static entities inside it.

**Buffer zone** is an inflation of the level's volume outside its boundaries used to detect whether a performer is inside the level.

**Performer** is simulation entity which may change levels during the simulation, such as a robot.

**Global entities** are present on all levels. They will be duplicated across all simulation runners.

**Default level** handles all entities that are not within any other levels.

**Network manager** controls the flow of information in a simulation distributed across processes.

**Event manager** manages events that can be sent across systems and the server. Plugins can operate events.

**Simulation runner** runs a whole world or some levels of a world, but no more than 1 world. It has a single ECM, an event manager, a network manager if the simulation is distributed, and a set of systems.

**Server** is Gazebo Sim's entry point that is responsible for loading an SDF file and instantiating a simulation runner per world.

## 1.2 Server Configuration

Servers are configured via SDF files. Most functionality on Gazebo is provided by plugins. The plugins can be defined in:

* `<plugin>` elements inside an SDF file.
* Path defined by the `GZ_SIM_SERVER_CONFIG_PATH` environment variable.
* The default configuration file at `$HOME/.gz/sim/<#>/server.config`, where `<#>` is Gazebo's major version.




## 1.3



# CLI Tools

Start a gazebo sim:

```shell
gz sim -s <path-to-sdf-file>
```

Parameters:

* `-s <path>` or `--sdf <path>` : Path to the SDF file to load.
* `-p <engine-name>` or `--physics <engine-name>` : Name of the physics engine to use. Default is `ign-physics`.
* `-v <level>` or `--verbose<level>` : Enable verbose output.
* `-r`: Run the simulation in real-time.




