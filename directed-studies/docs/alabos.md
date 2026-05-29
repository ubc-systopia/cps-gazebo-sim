# General Notes
* Must maximize the utilization rate of the autonomous laboratory
* Should be able to run experiments in parallel (concurrency)
* Graph-based experimental workflow
  * tasks are the nodes
  * task dependencies are the edges
* AlabOS requires database and parallel programming knowledge
* Status monitoring and notification system built-in -> GUI
* MongoDB (NoSQL) backend
* In AlabOS, a laboratory is represented as a combination of...
  * samples
  * devices
  * tasks
  * experiment entities
* Each entity record maintained in a separate collection in a MongoDB instance and updated during runtime
* Sample entity:
  * name
  * position
  * other metadata specified during submission
* Samples have positions
* When submitting an experiment, the operator can specify a directed acyclic graph (DAG) of tasks to be performed on each sample
* An experiment can contain multiple samples and tasks
  * A task can accept one ore more samples as input depending on capacity
* Four manager processes used to monitor and manipulate the status of the physical laboratory at different levels:
  * experiments
  * tasks
  * devices
  * resources
* task manager is alerted when a task is completed or an error encountered
* All tasks defined as python objects
* There is a simulation mode
* Priority numbers are used to establish ranking between resources requests
  * Priority can range from 1-100, but the default is 20
* Requests are first ranked by priority, then submission time
* Task managers do not communicate directly with the physical equipment but rather through RPC (remote procedure call)
* Submission API
* The python appears linear, but the backend turns this into a graph-based architecture