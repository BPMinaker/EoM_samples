function input_full_car_a_arm_pushrod(; u = 10.0, a = 1.2, b = 1.3, cf = 40000, cr = 40000, m = 1400, Ix = 800, Iy = 2000, Iz = 2200, r = 0.3, tw = 1.5, cs = 2000, ks = 60000, kt = 150000, ct = 100, flex = false, kba = 1e6, kbr = 6e6, cba = 1e4, cbr = 6e4)

    the_system = mbd_system("Full Car A-Arm Pushrod")

    item = body("Chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, Iz]
    item.products_of_inertia = [0.0, 0.0, 0.0]
    item.location = [0.0, 0.0, 0.5]
    item.velocity = [u, 0.0, 0.0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    susp!(the_system; str = "LF ", front = true, a, tw, r, u, cs, ks, flex, kba, kbr, cba, cbr)
    tire!(the_system; str = "LF ", front = true, a, tw, cf, kt, ct, u)
    drive!(the_system; str = "LF ", front = true, a, tw, r)

    susp!(the_system; a = -b, str = "LR ", front = false, tw, r, u, cs, ks, flex, kba, kbr, cba, cbr)
    tire!(the_system; a = -b, cf = cr, str = "LR ", front = false, tw, kt, ct, u)
    drive!(the_system; str = "LR ", front = false, a = -b, tw, r)


    item = spring("Anti-roll bar")
    item.body[1] = "LF Anti-roll arm"
    item.body[2] = "RF Anti-roll arm"
    item.location[1] = [a + 0.05, 0.25, r + 0.05]
    item.location[2] = [a + 0.05, -0.25, r + 0.05]
    item.stiffness = 10.0
    item.damping = 0.0
    item.twist = 1
    add_item!(item, the_system)


    # roll centre constraints
    #=
    item=rigid_point("LF RC")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location=[a,tw/2,0]
    item.forces=3
    item.moments=0
    add_item!(item,the_system)

    item=rigid_point("LR RC")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location=[-b,tw/2,0]
    item.forces=3
    item.moments=0
    add_item!(item,the_system)
    =#

    item = actuator("u_LF")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, 0]
    item.location[2] = [a, tw / 2, -0.1]
    item.gain = kt
    item.rate_gain = ct
    item.units = "m"
    item.desc = "LF Ground profile"
    add_item!(item, the_system)


    item = actuator("u_LR")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tw / 2, 0]
    item.location[2] = [-b, tw / 2, -0.1]
    item.gain = kt
    item.rate_gain = ct
    item.units = "m"
    item.desc = "LR Ground profile"
    add_item!(item, the_system)


    ####% Reflect all LF or LR items in y axis
    mirror!(the_system)


    # add sensors
    item = sensor("z_LFc")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    item.desc = "Chassis motion"
    add_item!(item, the_system)

    item = sensor("z_LFc-z_LF")
    item.body[1] = "Chassis"
    item.body[2] = "LF Wheel+hub"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    item.desc = "Suspension travel"
    add_item!(item, the_system)

    item = sensor("z_LF-u_LF")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.actuator = "u_LF"
    item.units = "m"
    item.desc = "Tire compression"
    add_item!(item, the_system)


    #=
    item = actuator("L")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0,0,0.5]
    item.location[2] = [0.1,0,0.5]
    item.twist = true
    item.gain = 1000
    add_item!(item,the_system)

    item = sensor("ϕ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0,0,0.5]
    item.location[2] = [0.1,0,0.5]
    item.twist = true
    item.gain = 180 / π
    add_item!(item,the_system)
    =#

    the_system

end ## Leave
