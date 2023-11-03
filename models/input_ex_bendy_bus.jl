function input_ex_bendy_bus(; u = 10, af = 2.22, bf = 3.68, c1 = 166000, c2 = 120000, mf = 15873, Jf = 136000, d = 5.57, ar = 4.10, br = 0, c3 = 152000, mr = 17452, Jr = 81000, k = 50000, Γ = 5000)

    ## The bus model from:
    # IAVSD_2023 conference paper: Stabilization of articulated buses through hydraulic joint control: a feasibility study, by DeFelice, Mercantini, Schramm, and Sorrentino
    ## af = front axle to cg
    ## bf = rear axle to cg
    ## d = hitch to front cg
    ## ar = hitch to rear cg
    ## br = rear axle to rear cg
    ## mf = front mass
    ## Jf = front inertia
    ## mr = rear mass
    ## Jr = rear inertia
    ## c1 = front cornering stiffness (total)
    ## c2 = rear cornering stiffness (total)
    ## c3 = trailer cornering stiffness (total)

    the_system = mbd_system("Bendy Bus")

    if (u == 0)
        error("Speed must be non-zero.")
    end

    item = body("front")
    item.mass = mf
    item.moments_of_inertia = [0, 0, Jf]
    item.location = [0, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = body("trailer")
    item.mass = mr
    item.moments_of_inertia = [0, 0, Jr]
    item.location = [-d - ar, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    item = flex_point("front tire")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location = [af, 0, 0]
    item.damping = [c1 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("rear tire")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location = [-bf, 0, 0]
    item.damping = [c2 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("trailer tire")
    item.body[1] = "trailer"
    item.body[2] = "ground"
    item.location = [-d - ar - br, 0, 0]
    item.damping = [c3 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    ## Constrain truck to planar motion
    item = rigid_point("road")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 1
    item.moments = 2
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    ## Constrain truck to trailer with hinge
    item = rigid_point("hitch")
    item.body[1] = "front"
    item.body[2] = "trailer"
    item.location = [-d, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = flex_point("hitch")
    item.body[1] = "front"
    item.body[2] = "trailer"
    item.location = [-d, 0, 0]
    item.forces = 0
    item.moments = 1
    item.stiffness = [0, k]
    item.damping = [0, Γ]
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = load("drive")
    item.body = "front"
    item.location = [-bf, 0, 0]
    item.force = [1.204 * 8 * 1.15 * 0.5 * u^2, 0, 0]
    item.moment = [0, 0, 0]
    item.frame = "front"
 #   add_item!(item, the_system)

    item = load("aero")
    item.body = "trailer"
    item.location = [-d - ar, 0, 0]
    item.force = [-1.204 * 8 * 1.15 * 0.5 * u^2, 0, 0]
    item.moment = [0, 0, 0]
    item.frame = "trailer"
 #   add_item!(item, the_system)

    # constrain chassis in the forward direction
    # the left/right symmetry of the chassis tells us that the lateral and longitudinal motions are decoupled anyway
    # could use nhpoint instead of rigid here, but just gives another zero eigenvalue, which causes grief elsewhere due to repeated zero roots
    item = rigid_point("speed")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("δ_f")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location[1] = [af, 0, 0]
    item.location[2] = [af, 0.1, 0]
    item.gain = c1 * pi / 180
    item.units = "°"
    add_item!(item, the_system)

    # measure the yaw rate
    item = sensor("r")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, 0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = 180 / pi # radian to degree
    item.units = "°/s"
    add_item!(item, the_system)

    item = sensor("β")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u # radian to degree
    item.units = "°"
    add_item!(item, the_system)

    item = sensor("θ")
    item.body[1] = "front"
    item.body[2] = "trailer"
    item.location[1] = [-d, 0, 0]
    item.location[2] = [-d, 0, 0.1]
    item.twist = 1 # angular
    item.gain = 180 / π # radian to degree
    item.units = "°"
    add_item!(item, the_system)

    # measure the lateral acceleration in g
    item = sensor("a_y")
    item.body[1] = "front"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0]
    item.order = 3 # acceleration
    item.gain = 1 / 9.81 # g
    item.units = "ge"
    add_item!(item, the_system)

    the_system

end ## Leave
