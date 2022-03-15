# FSAC-sim
A simulation for FSAC(Formula Student Autonomous China), mainly focus on perception, motion planning and control.
  
  
## How To Run
```
python fsac-sim.py
```

## Perceptor
In the competition, we use both camera and LiDAR to detect cones of the fsac track. However, as this project mainly focus on the planning and control algorithms of the FSAC racecar. 
I simplify the perception pipeline, which means that cones within 15m in front of the car can be stably detected.

## Planner


## TODO
-[x] rrt planner  
-[x] cubic spline planner
-[ ] cubic spline in base_link


