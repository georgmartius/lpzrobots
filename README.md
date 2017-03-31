# LPZRobots -- a simulator for robotic experiments #

This is a 3D physics simulator that comes with a collection of algorithms, simulations, and tools
developed by the Robotics Group for Self-Organization of Control.

Contributing people: Georg Martius (<georg.playfulmachines.com>), Ralf Der, Frank Hesse, Frank Güttler, Jörn Hoffmann

Former Contributors: Antonia Siegert, Marcel Kretschmann, Dominic Schneider, Claus Stadler

## Documentation ##
see the project page: <http://robot.informatik.uni-leipzig.de/software/?lang=en>
and the online help: <http://robot.informatik.uni-leipzig.de/software/doc/html/index.html>

## Installation ##

  - We have a docker file in `dist_utils/docker` for installing it on ubuntu (provided by Sebastian Blaes).

  - There is also a "one click" installation script kindly provided by Lars Gröber: <https://github.com/Larsg7/lpzrobots-install-script>

  - Otherwise go for the instructions <http://robot.informatik.uni-leipzig.de/software/doc/html/index.html>

## Overview ##
It consists of the following directories:

  - selforg : controllers together with a small framework for using them,
      developed in the robotic group yielding at self-organized behavior for various kinds of machines.
  - ode_robots : physics simulator based on ODE
      (Open Dynamics Engine, see <http://www.ode.org>).
      This includes robots, obstacles, utilities, stuff for visualization with OSG
        (OpenSceneGraph, see <http://www.openscenegraph.org>) and so on.
  - guilogger : application that coordinates multiple gnuplot
      windows and allows for an interactive display of data that is sent per pipe from another processes. (will be started from ode_robots)
  - matrixvix : application for interactive display of changing matrix and vector data
  - configurator : a library implementing a GUI to change the parameters interactively
        which is otherwise done on the console
  - ga\_tools : genetic algorithms framework that can be used to together with
        ode_robots or for independent simulations (not well maintained) program
  - opende : directory with a snapshot of the open dynamics engine (release 0.11.1)
                  renamed to ode-dbl in order to avoid conflicts with packaged single
                  precision versions. It contains the capsule-box collision bugfix
                  which is upstream (in svn)
