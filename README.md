# EoM_samples

Note that in order to run these sample models, you should download a .zip of the this folder (under the green '<> Code' button above).  There is no need to download the EoM library itself from github - that will be done as part of your Julia setup: run Julia, hit ] to go to pkg> prompt, and enter: add https://github.com/BPMinaker/EoM.jl

These are a bunch of sample input files for EoM.  You can edit them for your use, but you should find here code for:
1. the spring mass damper model, to get started
2. the yaw plane model. A single body vehicle model aka the `bicycle' model, now with a driver model included
3. the truck and trailer model.  Like the yaw plane, but two bodies, hinged
4. quarter car model.  A classic for ride quality
5. bounce-pitch model.  Also great for introduction to ride quality
6. half car model.  More advanced ride quality model
7. full car model.  Like half car, but 7 dof
8. bicycle model.  An actual bicycle, the well studied Whipple model, see benchmark paper from Meijaard et al
9. drag_race model. Includes nonlinear aero and engine model effects
10. full_car_A_arm model.  A huge model with all the tricks, like full suspension, anti-roll bars, transient lateral tire model, parallel spring-damper in series with undamped stiff spring (to include bushing effects)
11. quarter_car_A_arm model.  As the name suggests, one corner of the full car model above
12. disk model. the simple rolling disk, well studied in the literature
13. pendulum model.  A single body supported by a massless wire, from Anderson's Engineering Dynamics
14. bricard model. A famous complicated 3D 5 body mechanism, has redundant constraints, but still moves, small motions only
15. rotor stator model.  A benchmark from the literature, see Negrut and Ortiz 2006 paper
16. spining top model.  A classic problem in dynamics
17. Euler beam element model. EoM allows you to include beams with or without mass in your models

