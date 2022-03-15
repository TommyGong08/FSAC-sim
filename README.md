# FSAC-sim  
A simulation for FSAC(Formula Student Autonomous China), mainly focus on perception, motion planning and control.  
  
  
## Support  
The following are the algorithms supported in this project. In the competition, we use both camera and LiDAR to detect cones of the fsac track. However, as this project mainly focus on the planning and control algorithms of the FSAC racecar, 
I simplify the perception pipeline, making sure that cones within 15m in front of the car can be stably detected.  
  
1.perception  
- [x] default  
  
2.planner  
- [x] rrt  
- [x] cubic_spline  
- [x] quintic_polynmial  

3.control   
- [x] dynamic_windows  
  
## How To Run  
```
python fsac-sim.py
```

## TODO  
1.planner  
- [ ] implement cubic spline in base_link  
- [ ] A*  
- [ ] frenet optimal  
- [ ] B Spline  

2.control  
- [ ] pure pursuit  
- [ ] mpc  
