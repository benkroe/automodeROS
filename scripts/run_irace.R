library(irace)

scenario_path <- "/home/ben/ros2_ws/automodeROS/scenario.txt"
sc <- readScenario(scenario_path)

# Force absolute paths
sc$parameterFile <- "/home/ben/ros2_ws/automodeROS/irace/parameters.txt"
sc$targetRunner <- "/home/ben/ros2_ws/automodeROS/target-runner"

# Provide instances directly (avoid trainInstancesDir/File issues)
sc$instances <- c("default_instance")
sc$trainInstancesDir <- ""
sc$trainInstancesFile <- ""

irace(scenario = sc)
