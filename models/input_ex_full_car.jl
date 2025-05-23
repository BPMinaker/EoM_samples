function input_ex_full_car(;
     u = 0,
    a = 1.189,
    b = 2.885 - 1.189,
    tf = 1.595,
    tr = 1.631,
    kf = 17000,
    kr = 19000,
    cf = 1000,
    cr = 1200,
    m = 16975 / 9.81,
    Ix = 818,
    Iy = 3267,
    kt = 180000,
    muf = 35,
    mur = 30
    )

    the_system = mbd_system("Full Car Model")

    # add one body representing the chassis
    item = body("chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, 0]  ## Only the Iy term matters here
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, 0.25]  ## Put cg at origin, but offset vertically to make animation more clear
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF unsprung")
    item.mass = muf
    item.location = [a, tf / 2, 0.1]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LR unsprung")
    item.mass = mur
    item.location = [-b, tr / 2, 0.1]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # front suspension
    item = flex_point("LF spring")
    item.body[1] = "chassis"
    item.body[2] = "LF unsprung"
    item.location = [a, tf / 2, 0.25] # front axle "a" m ahead of cg
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1] # spring acts in z direction
    item.stiffness = [kf, 0] # linear stiffness "kf" N/m; (torsional stiffness zero, not a torsion spring so has no effect
    item.damping = [cf, 0]
    add_item!(item, the_system)

    # rear suspension
    item = flex_point("LR spring")
    item.body[1] = "chassis"
    item.body[2] = "LR unsprung"
    item.location = [-b, tr / 2, 0.25]  ## Front axle "a" m ahead of cg
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]  ## Spring acts in z direction
    item.stiffness = [kr, 0]  ## Linear stiffness "kr" N/m; (torsional stiffness zero, not a torsion spring so has no effect
    item.damping = [cr, 0]
    add_item!(item, the_system)


    # tires
    item = flex_point("LF tire")
    item.body[1] = "LF unsprung"
    item.body[2] = "ground"
    item.stiffness = [kt, 0]
    item.damping = [0, 0]
    item.location = [a, tf / 2, 0.15]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = flex_point("LR tire")
    item.body[1] = "LR unsprung"
    item.body[2] = "ground"
    item.stiffness = [kt, 0]
    item.damping = [0, 0]
    item.location = [-b, tr / 2, 0.15]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)


    # suspension constraints
    item = rigid_point("LF susp")
    item.body[1] = "LF unsprung"
    item.body[2] = "chassis"
    item.location = [a, tf / 2, 0.1]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = rigid_point("LR susp")
    item.body[1] = "LR unsprung"
    item.body[2] = "chassis"
    item.location = [-b, tr / 2, 0.1]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # constrain to linear motion in z direction (bounce)
    item = rigid_point("road frc")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, 0.25]
    item.forces = 2
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # constrain to rotational motion around x and y axes (roll and pitch)
    item = rigid_point("road mmt")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location = [0, 0, 0.25]
    item.forces = 0
    item.moments = 1
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    mirror!(the_system)

    # force motion
    item = actuator("u_lf")
    item.body[1] = "LF unsprung"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0.25]
    item.location[2] = [a, tf / 2, 0]
    item.gain = kt
    item.units = "m"
    add_item!(item, the_system)

    # measure the bounce, pitch, and roll
    item = sensor("z_G")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, 0, 0]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("θ(a+b)")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, -0.1, 0.25]
    item.gain = a + b
    item.twist = 1
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("ϕ(t_f)")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [-0.1, 0, 0.25]
    item.gain = tf
    item.twist = 1
    item.units = "m"
    add_item!(item, the_system)

    the_system

end
