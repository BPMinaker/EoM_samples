function input_ex_yaw_plane_flex_roll_steer(; u = 10.0, a = 1.189, b = 2.885 - 1.189, cf = 80000.0, cr = 80000.0, ptf = 0, ptr = 0, ms = 16975 / 9.81, muf = 0, mur = 0, Izs = 3508.0, lf=0.02, lr=0.02, kf=1E8, kr=1E8, ef=0, er=0, k_phi=0.005)

    # The classic yaw plane model
    # a = front axle to truck cg
    # b = rear axle to truck cg
    # m = mass
    # I = yaw inertia
    # cf = front cornering stiffness (total)
    # cr = rear cornering stiffness (total)

    the_system = mbd_system("Yaw Plane Vehicle w Flex and Roll Steer")

    if (u == 0)
        error("Speed must be non-zero!")
    end

    # add one rigid body
    item = body("chassis")
    item.mass = ms
    item.moments_of_inertia = [0, 0, Izs]
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, 1/ms]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = body("front wheel")
    item.mass = muf
    item.moments_of_inertia = [0, 0, 0]
    item.products_of_inertia = [0, 0, 0]
    item.location = [a, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = body("front axle")
    item.mass = 0
    item.moments_of_inertia = [0, 0, 0]
    item.products_of_inertia = [0, 0, 0]
    item.location = [a, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = body("rear wheel")
    item.mass = mur
    item.moments_of_inertia = [0, 0, 0]
    item.products_of_inertia = [0, 0, 0]
    item.location = [-b, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = body("rear axle")
    item.mass = 0
    item.moments_of_inertia = [0, 0, 0]
    item.products_of_inertia = [0, 0, 0]
    item.location = [-b, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)


    item = rigid_point("front steer hinge")
    item.body[1] = "front axle"
    item.body[2] = "chassis"
    item.location = [a - ptf, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [cos(ef), 0, sin(ef)]
    add_item!(item, the_system)

    item = rigid_point("rear steer hinge")
    item.body[1] = "rear axle"
    item.body[2] = "chassis"
    item.location = [-b - ptr, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [cos(er), 0, -sin(er)]
    add_item!(item, the_system)


    item = rigid_point("front axle")
    item.body[1] = "front axle"
    item.body[2] = "ground"
    item.location = [a, 0, 0]
    item.forces = 0
    item.moments = 1
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("rear axle")
    item.body[1] = "rear axle"
    item.body[2] = "ground"
    item.location = [-b, 0, 0]
    item.forces = 0
    item.moments = 1
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = flex_point("roll stiffness")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 0
    item.moments = 1
    item.axis = [1, 0, 0]
    item.stiffness = [0, 1/k_phi]
    add_item!(item, the_system)


    item = rigid_point("front flex hinge")
    item.body[1] = "front wheel"
    item.body[2] = "front axle"
    item.location = [a + lf, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = rigid_point("rear flex hinge")
    item.body[1] = "rear wheel"
    item.body[2] = "rear axle"
    item.location = [-b + lr, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = flex_point("front flex")
    item.body[1] = "front axle"
    item.body[2] = "front wheel"
    item.location = [a, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    item.stiffness = [kf, 0]
    add_item!(item, the_system)

    item = flex_point("rear flex")
    item.body[1] = "rear axle"
    item.body[2] = "rear wheel"
    item.location = [-b, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    item.stiffness = [kr, 0]
    add_item!(item, the_system)


    # add a damping, to connect our body to ground, aligned with y-axis (front tire)
    item = flex_point("front tire")
    item.body[1] = "front wheel"
    item.body[2] = "ground"
    item.location = [a - ptf, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    item.damping = [cf / u, 0]
    add_item!(item, the_system)

    # rear tire
    item = flex_point("rear tire")
    item.body[1] = "rear wheel"
    item.body[2] = "ground"
    item.location = [-b - ptr, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    item.damping = [cr / u, 0]
    add_item!(item, the_system)

    # add an actuator to apply the steering force
    item = actuator("δ_f")
    item.body[1] = "front wheel"
    item.body[2] = "ground"
    item.location[1] = [a - ptf, 0, 0]
    item.location[2] = [a - ptf, 0.1, 0]
    item.gain = cf * π / 180 # degree to radian
    item.units = "°"
    add_item!(item, the_system)

    # rear wheel steer, off by default
    item = actuator("δ_r")
    item.body[1] = "rear wheel"
    item.body[2] = "ground"
    item.location[1] = [-b - ptr, 0, 0]
    item.location[2] = [-b - ptr, 0.1, 0]
    item.gain = cr * π / 180
    item.units = "°"
    #add_item!(item,the_system)


   # constrain bounce
   item = rigid_point("bounce")
   item.body[1] = "chassis"
   item.body[2] = "ground"
   item.location = [0, 0, 0]
   item.forces = 1
   item.moments = 0
   item.axis = [0, 0, 1]
   add_item!(item, the_system)

   # constrain pitch
   item = rigid_point("pitch")
   item.body[1] = "chassis"
   item.body[2] = "ground"
   item.location = [0, 0, 0]
   item.forces = 0
   item.moments = 1
   item.axis = [0, 1, 0]
   add_item!(item, the_system)


    # constrain chassis in the forward direction
    # the left/right symmetry of the chassis tells us that the lateral and longitudinal motions are decoupled anyway
    # could use nhpoint instead of rigid here, but just gives another zero eigenvalue, which causes grief elsewhere due to repeated zero roots
    item = rigid_point("speed")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # measure the yaw rate
    item = sensor("r")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, 0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = 180 / π # radian to degree
    item.units = "°/s"
    add_item!(item, the_system)

    # measure the body slip angle
    item = sensor("β")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u # radian to degree
    item.units = "°"
    add_item!(item, the_system)

    # measure the understeer angle
    item = sensor("α_u")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, 0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = -180 * (a + b) / π / u # radian to degree
    item.actuator = "δ_f"
    item.actuator_gain = 1 # input is already in degrees
    item.units = "°"
    add_item!(item, the_system)


    # measure the front flex angle
    item = sensor("γ_f")
    item.body[1] = "front wheel"
    item.body[2] = "front axle"
    item.location[1] = [a, 0, 0]
    item.location[2] = [a, 0, 0.1]
    item.twist = true
    item.gain = 180 / π # radian to degree
    item.units = "°"
    add_item!(item, the_system)


    # measure the rear flex angle
    item = sensor("γ_r")
    item.body[1] = "rear wheel"
    item.body[2] = "rear axle"
    item.location[1] = [-b, 0, 0]
    item.location[2] = [-b, 0, 0.1]
    item.twist = true
    item.gain = 180 / π # radian to degree
    item.units = "°"
    add_item!(item, the_system)


    # measure the front slip angle
    item = sensor("α_f")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [a, 0, 0]
    item.location[2] = [a, 0.1, 0]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u # radian to degree
    item.actuator = "δ_f"
    item.actuator_gain = -1 # input is already in degrees
    item.units = "°"
    add_item!(item, the_system)


    # measure the rear slip angle
    item = sensor("α_r")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [-b, 0, 0]
    item.location[2] = [-b, 0.1, 0]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u # radian to degree
#    item.actuator = "δ_r"
#    item.actuator_gain = -1 # input is already in degrees
    item.units = "°"
    add_item!(item, the_system)


    item = sensor("ϕ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0.1, 0, 0]
    item.location[2] = [0, 0, 0]
    item.twist = true
    item.gain = 180 / π # radian to degree
    item.units = "°"
    add_item!(item, the_system)


    # measure the steady state lat acc
    item = sensor("ru")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, 0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain =  u / 9.81 # radian to degree
    item.units = "ge"
    add_item!(item, the_system)

    the_system

end
