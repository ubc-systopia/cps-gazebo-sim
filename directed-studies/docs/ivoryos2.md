# General Notes
* An experiment is comprised of modular processes.
  * Processes are the building blocks of the experiment.
  * Prep and clean up would be considered processes
* When you run an experiment, this is considered an experiment run. If you have something that must take place many times, you can instead repeat the processes within the experiment.
* There are predefined pieces of hardware. To create a hardware device in a particular lab, you create an instance of that piece of hardware. Robot arms are included as part of the hardware. Each piece of hardware is its own class.
* During an experiment, there are logs that appear. There is always one log file associated with one experiment run.
* In the new design there should be a way to map different semantic naming for locations to the actual cartesian coordinates
  * There should be locations for an object and sub-locations. An example of this would be the location of a tray and the different locations within the tray.
* Do we want the new design to be fully abstracted or still be a way for the user to interface with their python scripts that control the lab?
* If we want users to be able to use their own python scripts, we need to create a very natural way for this to occur. This means that it must be very straightforward how to match the schema of the overarching application
  * i.e. each public method corresponds to one task

# Design
* Experiment
  * Comprised of processes
  * Information about how many times certain processes should occur
  * Order of processes
* Process
  * Modular building blocks for an experiment
* Tasks
  * Atomic actions that make up processes
* Experiment runs
  * Includes logs and other metadata about a particular experiment
* Logs
* Hardware/instruments
  * List of different configurable instruments that may be used in a lab
* Location
  * Each instrument should have a location

# Questions
* Do we want users to have their own python scripts or should everything be through the UI interface?
  * **It would be a no-code design for the chemists, but there would still need to be some code in order to connect devices and begin the process**
* Should there be the ability for both -> this would further decouple the low-level layer from the higher-level layer
* 