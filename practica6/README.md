# Vigilanti Mission Planner

Vigilanti is a robocomp project that allows a final user to fleet robots to patrol over a place.

It's conformed by 3 components:

  - Movement Controller.
  - Camera Controller.
  - Supervisor.

Those 3 components work together in order to perform an iterative process where the robot go lokking for a tag, and when the target tag is seen, he goes into another, this process is full cyclic.

### Features!

  - Mission planner come with an easy to launch script. Just provide the working directory where components are, and magic  starts!
  - Telegram integration, tolds you when a tag is found.
  - MultiRobot supporting, just config multiple robots with different ports and start the launching script providing the different folders.

### Dependencies
  -  Linux (preferably Ubuntu 16.04)
  - C++
  - Qt4.8
  - Git
  - GitHub
  - KDevelop
  - ZeroC's Ice Internet Communication Middleware
  - OpenSceneGraph  3D rendering engine
  - Apriltags, augmented reality marks 
  - RoboLab's RoboComp
  - https://github.com/reo7sp/tgbot-cpp (For telegram integration)


### Running

Mission Planner requires Robocomp framework and the dependencies above to run. Compile after having installing all dependecies and then, go to practicasRobotica folder and type:

```sh
$ ./startSupervisor.sh practicasRobotica/practica6
```

When having multiple environments...

```sh
$ ./startSupervisor.sh 'ANOTHER WORKING DIR'
```

### More

You can find the entire development paper as https://github.com/jgallardst/practicasRobotica/blob/master/FinalRoboticsMemories.pdf

