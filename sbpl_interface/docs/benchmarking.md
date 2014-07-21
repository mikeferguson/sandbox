## Benchmarking

Not very scientific yet -- need to create actual scenarios, but this is
basically a quick comparison of the differences for two somewhat similar
number of expansions:

|            | moveit | sbpl_arm_planner |
|------------|:------:|:----------------:|
|expansions  | 100    | 133              |
|setup time  | 0.19s  | 0.050s*          |
|plan time   | 0.366s | 0.140s           |
|bfs time    | 0.06s  | 0.073s           |
|total time  | 0.538s | 0.162s           |

*Not entirely clear when setup starts/stops -- should probably update the
planner to have the exact same performance checks. 

Need to output the following from moveit: solution cost, initial/final epsilon

Need to output the following from sbpl_arm_planner: setup time, collision checking time total
