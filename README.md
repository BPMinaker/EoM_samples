# EoM_samples

Note that in order to run these sample models, you should download a .zip of the this folder (under the green '<> Code' button above).  There is no need to download the EoM library itself from github - that will be done as part of your Julia setup: navigate to this folder after you download and unzip it (you can do this on the command line, or if you are using VS Code, make sure you use File, Open Folder), run Julia (or launch a REPL if you are in VS Code), hit ] to go to pkg> prompt.  If the prompt says v1.11> or something like that, enter: activate . to activate the current folder (the prompt should change to EoM_samples>), then enter: instantiate

These are a bunch of sample input files for EoM.  You can edit them for your use, but you should find here code for:
1. the spring mass damper model, to get started
2. the yaw plane model - a single body vehicle model aka the `bicycle' model, now with a driver model included
3. the truck and trailer model - ike the yaw plane, but two bodies, hinged
4. quarter car model - a classic for ride quality
5. bounce-pitch model - also great for introduction to ride quality
6. half car model - more advanced ride quality model
7. full car model - like half car, but 7 dof
8. bicycle model - an actual bicycle, the well studied rigid rider Whipple model, see benchmark paper from Meijaard et al
9. drag_race model - includes nonlinear aero and engine model effects
10. full_car_A_arm model - a huge model with all the tricks, like full suspension, anti-roll bars, transient lateral tire model, parallel spring-damper in series with undamped stiff spring (to include bushing effects)
11. quarter_car_A_arm model - as the name suggests, one corner of the full car model above
12. disk model - the simple rolling disk, well studied in the literature
13. pendulum models - single body supported by a massless wire, from Anderson's Engineering Dynamics, plus a few variants
14. five bar pendulum from the iftomm multibody benchmark webpage produced by Francisco González
15. bricard model - famous complicated 3D 5 body mechanism, has redundant constraints, but still moves, small motions only
16. rotor stator model - benchmark from the literature, see Negrut and Ortiz 2006 paper
17. spining top model - a classic problem in dynamics
18. Euler beam element model - EoM allows you to include beams with or without mass in your models
19. landing gear shimmy model - a benchmark from the literature, see Schwab's paper
20. ziegler column model - a follwer load buckling model
21. articulated bus model - like truck and trailer but each body only has one axle - benchmark from IAVSD conference paper by De Felice, Mercantini, Schramm, Sorrentino
22. a laterally loaded A-arm suspension model to explore the effect of kinematics on lateral-vertical coupling and the distinction between the roll centre, instant centre, and the contact patch path centre of curvature - full details in the IAVSD conference paper I wrote with Jen Johrendt

and many others...
