function input_ex_quarter_car(; mu = 50, ms = 500, ks = 18000, kt = 180000, cs = 1500, ct = 500)

    # A 'quarter-car' model, two bodys, constrained to ground, allowing translation in the z axis only, with two point springs connecting them.  The point spring has stiffness and damping defined in translation along the z axis only.  An actuator connects the sprung mass to the ground as well, to provide input forces.
    ##  Note that the ground body is pre-defined.

    the_system = mbd_system("Quarter Car Model")

    # Add the unsprung mass, along the z-axis
    item = body("unsprung")
    item.mass = mu
    item.location = [0, 0, 0.3]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add another identical rigid body, along the z-axis
    item = body("sprung")
    item.mass = ms
    item.location = [0, 0, 0.6]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add a spring, with no damping, to connect our unsprung mass to ground, aligned with z-axis
    item = flex_point("tire")
    item.body[1] = "unsprung"
    item.body[2] = "ground"
    item.stiffness = [kt, 0]
    item.damping = [ct, 0]
    item.location = [0, 0, 0.15]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # Add another spring, with damping, to connect our sprung and unsprung mass
    item = flex_point("susp")
    item.body[1] = "sprung"
    item.body[2] = "unsprung"
    item.stiffness = [ks, 0]
    item.damping = [cs, 0]
    item.location = [0, 0, 0.45]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # Constrain unsprung mass to translation in z-axis, no rotation
    item = rigid_point("slider one")
    item.body[1] = "unsprung"
    item.body[2] = "ground"
    item.location = [0, 0, 0.3]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # Constrain sprung mass to translation in z-axis, no rotation
    item = rigid_point("slider two")
    item.body[1] = "sprung"
    item.body[2] = "ground"
    item.location = [0, 0, 0.6]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # Add external force between unsprung mass and ground (represents ground motion)
    item = actuator("z_g")
    item.body[1] = "unsprung"
    item.body[2] = "ground"
    item.gain = kt
    item.rate_gain = ct
    item.location[1] = [0.05, 0, 0.3]
    item.location[2] = [0.05, 0, 0]
    item.units = "m"
    item.desc = "Ground profile"
    add_item!(item, the_system)

    # Add measure between ground and sprung mass
    item = sensor("z_s")
    item.body[1] = "sprung"
    item.body[2] = "ground"
    item.location[1] = [0, 0.05, 0.6]
    item.location[2] = [0, 0.05, 0]
    item.units = "m"
    item.desc = "Sprung mass motion"
    add_item!(item, the_system)

    # Add measure between sprung mass and unsprung mass
    item = sensor("z_s-z_u")
    item.body[1] = "unsprung"
    item.body[2] = "sprung"
    item.location[1] = [0.1, 0, 0.3]
    item.location[2] = [0.1, 0, 0.6]
    item.units = "m"
    item.desc = "Suspension travel"
    add_item!(item, the_system)

    # Add measure between ground and unsprung mass
    item = sensor("z_u-z_g")
    item.body[1] = "unsprung"
    item.body[2] = "ground"
    item.actuator = "z_g"
    item.location[1] = [0.1, 0, 0.3]
    item.location[2] = [0.1, 0, 0]
    item.units = "m"
    item.desc = "Tire compression"
    add_item!(item, the_system)

    item = sensor("f_s")
    item.body[1] = "unsprung"
    item.body[2] = "sprung"
    item.location[1] = [0.1, 0, 0.3]
    item.location[2] = [0.1, 0, 0.6]
    item.units = "N"
    item.order = 1
    item.gain = ks
    item.desc = "Suspension spring force"
    add_item!(item, the_system)

    item = sensor("f_d")
    item.body[1] = "unsprung"
    item.body[2] = "sprung"
    item.location[1] = [0.1, 0, 0.3]
    item.location[2] = [0.1, 0, 0.6]
    item.units = "N"
    item.order = 2
    item.gain = cs
    item.desc = "Suspension damper force"
    add_item!(item, the_system)

    item = sensor("f_i")
    item.body[1] = "sprung"
    item.body[2] = "ground"
    item.location[1] = [0, 0.05, 0.6]
    item.location[2] = [0, 0.05, 0]
    item.units = "N"
    item.gain = ms
    item.order = 3
    item.desc = "Sprung mass inertial force"
    add_item!(item, the_system)

    the_system

end
