# EoM_samples

These are a bunch of sample input files for EoM.  You can edit them for your use, but you should find here code for:
1. the spring mass damper model, to get started
2. the yaw plane model. A single body vehicle model aka the `bicycle' model
3. the truck and trailer model.  Like the yaw plane, but two bodies, hinged
4. quarter car model.  A classic for ride quality
5. bounce-pitch model.  Also great for introduction to ride quality
6. half car model.  More advanced ride quality model.
7. full car model.  Like half car, but 7 dof.
8. bicycle model.  An actual bicycle, the well studied Whipple model
9. drag_race model. The only one that requires the OrdinaryDiffEq library because it includes nonlinear aero and engine model effects
10. full_car_A_arm model.  A huge model with all the tricks, like full suspension, anti-roll bars, transient lateral tire model, parallel spring-damper in series with undamped stiff spring (to include bushing effects).
