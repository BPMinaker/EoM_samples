function input_full_car_rc(;
    m = 1800, # mass
    u = 20, # speed
    a = 1.5, # wheelbase, front
    b = 1.5,
    tf = 1.6, # track width, front
    tr = 1.6,
    kf = 30000, # suspension stiffness, front
    kr = 30000,
    cf = 2500, # suspension damping, front
    cr = 2500,
    krf = 500, # anti-roll stiffness, front
    krr = 500,
    muf = 50, # unsprung mass, front
    mur = 50,
    hf = 0.2, # roll centre height, front
    hr = 0.2,
    hG = 0.5, # mass centre height
    cfy = 40000, # cornering stiffness, front
    cry = 40000,
    Ix = 818, # moments of inertia
    Iy = 3267,
    Iz = 3508,
    kt = 180000, # tire vertical stiffness
    r = 0.3 # wheel radius
    )

    the_system = mbd_system("Full Car Model with Swing Axles")

    # add one body representing the chassis
    item = body("chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, Iz]
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, hG]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF wheel")
    item.mass = muf
    item.location = [a, tf / 2, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LR wheel")
    item.mass = mur
    item.location = [-b, tr / 2, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF axle")
    item.mass = 0
    item.location = [a, tf / 2 - 0.15, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LR axle")
    item.mass = 0
    item.location = [-b, tr / 2 - 0.15, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = rigid_point("fixed speed")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, hG]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LF wheel bearing")
    item.body[1] = "LF wheel"
    item.body[2] = "LF axle"
    item.location = [a, tf / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR wheel bearing")
    item.body[1] = "LR wheel"
    item.body[2] = "LR axle"
    item.location = [-b, tr / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LF wheel, X")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location = [a, tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LR wheel, X")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location = [-b, tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # suspension constraints
    item = rigid_point("LF susp")
    item.body[1] = "LF axle"
    item.body[2] = "chassis"
    item.location = [a, 0, hf]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LR susp")
    item.body[1] = "LR axle"
    item.body[2] = "chassis"
    item.location = [-b, 0, hr]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # anti-roll
    item = body("LF anti-roll arm")
    item.location = [a - 0.2, tf / 2 - r / 2, r - 0.1]
    add_item!(item, the_system)

    item = body("LR anti-roll arm")
    item.location = [-b + 0.2, tr / 2 - r / 2, r - 0.1]
    add_item!(item, the_system)

    item = rigid_point("LF anti-roll hinge")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "chassis"
    item.location = [a - 0.2, tf / 2 - r / 2, r - 0.1]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR anti-roll hinge")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "chassis"
    item.location = [-b + 0.2, tr / 2 - r / 2, r - 0.1]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LF drop link")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "LF axle"
    item.location[1] = [a, tf / 2 - r / 2, r - 0.1]
    item.location[2] = [a, tf / 2 - r / 2, r]
    add_item!(item, the_system)

    item = link("LR drop link")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "LR axle"
    item.location[1] = [-b, tr / 2 - r / 2, r - 0.1]
    item.location[2] = [-b, tr / 2 - r / 2, r]
    add_item!(item, the_system)

    # anti-roll bars
    # note that the right side bodies will come from the mirror
    item = spring("F anti-roll")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "RF anti-roll arm"
    item.location[1] = [a - 0.2, tf / 2 - 0.2, r - 0.1]
    item.location[2] = [a - 0.2, -tf / 2 + 0.2, r - 0.1]
    item.stiffness = krf
    item.twist = 1
    item.preload = 0
    add_item!(item, the_system)

    item = spring("R anti-roll")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "RR anti-roll arm"
    item.location[1] = [-b + 0.2, tr / 2 - 0.2, r - 0.1]
    item.location[2] = [-b + 0.2, -tr / 2 + 0.2, r - 0.1]
    item.stiffness = krr
    item.twist = 1
    item.preload = 0
    add_item!(item, the_system)


    # front suspension
    item = spring("LF spring")
    item.body[1] = "LF axle"
    item.body[2] = "chassis"
    item.location[1] = [a, tf / 2 - 0.15, r]
    item.location[2] = [a, tf / 2 - 0.15, 2 * r + 0.1]
    item.stiffness = kf
    item.damping = cf
    add_item!(item, the_system)

    # rear suspension
    item = spring("LR spring")
    item.body[1] = "LR axle"
    item.body[2] = "chassis"
    item.location[1] = [-b, tr / 2 - 0.15, r]
    item.location[2] = [-b, tr / 2 - 0.15, 2 * r + 0.1]
    item.stiffness = kr
    item.damping = cr
    add_item!(item, the_system)


    # tire vertical stifness
    item = flex_point("LF tire, Z")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.stiffness = [kt, 0]
    item.damping = [0, 0]
    item.location = [a, tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR tire, Z")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.stiffness = [kt, 0]
    item.damping = [0, 0]
    item.location = [-b, tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    # tire cornering stiffness
    item = flex_point("LF tire, Y")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.damping = [cfy / u, 0]
    item.location = [a, tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR tire, Y")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.damping = [cry / u, 0]
    item.location = [-b, tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # tire lateral force
    item = actuator("Y_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire lateral force"
    add_item!(item, the_system)

    item = sensor("N_lf")
    item.body[1] = "ground"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [1, 0, 0]
    item.units = "N*m"
    item.actuator = "Y_lf"
    item.actuator_gain = a
    item.desc = "Yaw moment"
    add_item!(item, the_system)

    item = actuator("Y_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire lateral force"
    add_item!(item, the_system)

    item = sensor("N_lr")
    item.body[1] = "ground"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [1, 0, 0]
    item.units = "N*m"
    item.actuator = "Y_lr"
    item.actuator_gain = -b
    item.desc = "Yaw moment"
    add_item!(item, the_system)

    # tire measure vertical force
    item = sensor("Z_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.gain = kt
    item.location[1] = [a, tf / 2, 0.1]
    item.location[2] = [a, tf / 2, 0]
    item.units = "N"
    item.desc = "Tire vertical force"
    add_item!(item, the_system)

    item = sensor("Z_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.gain = kt
    item.location[1] = [-b, tr / 2, 0.1]
    item.location[2] = [-b, tr / 2, 0]
    item.units = "N"
    item.desc = "Tire vertical force"
    add_item!(item, the_system)

    # tire, measure slip angle
    item = sensor("α_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.gain = 1 / u
    item.units = "rad"
    item.desc = "Tire slip angle"
    add_item!(item, the_system)

    item = sensor("α_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.gain = 1 / u
    item.units = "rad"
    item.desc = "Tire slip angle"
    add_item!(item, the_system)

    mirror!(the_system) # note that the mirror can't go any further without adressing the change in the location in the sequence of items in the main file

    # 13
    item = sensor("r")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, 0, 0]
    item.twist = 1
    item.order = 2
    item.gain = 180 / pi
    item.units = "°/s"
    item.desc = "Yaw rate"
    add_item!(item, the_system)

    # measure the bounce, pitch, and roll
    # 14
    item = sensor("z_G")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, 0, 0]
    item.gain = 1000
    item.units = "mm"
    item.desc = "Vertical displacement"
    add_item!(item, the_system)

    #15
    item = sensor("ϕ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [-0.1, 0, 0.25]
    item.gain = 180 / pi
    item.twist = 1
    item.units = "°"
    item.desc = "Roll angle"
    add_item!(item, the_system)

    #16
    item = sensor("θ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, -0.1, 0.25]
    item.gain = 180 / pi
    item.twist = 1
    item.units = "°"
    item.desc = "Pitch angle"
    add_item!(item, the_system)

    #17
    item = sensor("β")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, -0.1, 0.25]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / pi / u # radian to degree
    item.units = "°"
    item.desc = "Slip angle"
    add_item!(item, the_system)

    #18
    item = sensor("α_u-δ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, -0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = -180 * (a + b) / π / u # radian to degree
    item.units = "°"
    item.desc = "Understeer term"
    add_item!(item, the_system)

    #19
    item = sensor("ru")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, -0.1]
    item.gain = u
    item.order = 2 # velocity
    item.twist = 1
    item.units = "m/s/s"
    item.desc = "Lateral acceleration (steady state)"
    add_item!(item, the_system)

    # note that the y location will not reach steady state with constant delta input, so adding the sensor will give an error if the steady state gain is computed, but is included so that a time history can be computed
    item = sensor("y")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.units = "m"
    item.desc = "Lateral position"
    add_item!(item, the_system)

    # also won't reach steady state with constant delta input
    item = sensor("ψ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, -0.1]
    item.twist = 1 # angular
    item.gain = 180 / π
    item.units = "°"
    item.desc = "Yaw angle"
    add_item!(item, the_system)

    the_system

end

