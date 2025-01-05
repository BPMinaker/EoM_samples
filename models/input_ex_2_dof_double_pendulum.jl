function input_double_pendulum(; m1 = 1.0, m2 = 1.0, l1 = 1.0, l2 = 2.0, l3 = 1.5)

    # Define the double pendulum system
    the_system = mbd_system("Double Pendulum")

    g = 9.807

    # Add the first pendulum body
    item = body("pendulum1")
    item.mass = m1
    item.location = [0, 0, -l1]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the second pendulum body
    item = body("pendulum2")
    item.mass = m2
    item.location = [0, 0, -l2]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Constrain the first pendulum to rotate about the pivot point
    item = rigid_point("joint1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Constrain the second pendulum to rotate about the end of the first pendulum
    item = rigid_point("joint2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location = [0, 0, -l3]
    item.axis = [0, 1, 0]
    item.forces = 3
    item.moments = 2
    add_item!(item, the_system)

    item = actuator("M1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -l1]
    item.location[2] = [0, -0.1, -l1]
    item.twist = true
    item.units = "M*m"
    add_item!(item, the_system)

    item = actuator("M2")
    item.body[1] = "pendulum2"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -l2]
    item.location[2] = [0, -0.1, -l2]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    # Define sensors for angles and angular velocities
    item = sensor("m1gl1θ1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.twist = true
    item.gain = m1 * g * l1
    item.units = "N*m"
    add_item!(item, the_system)

    # Note relative coordinate output
    item = sensor("m2gl4θ2")
    item.body[1] = "pendulum2"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -l3]
    item.location[2] = [0, -0.1, -l3]
    item.twist = true
    item.gain = m2 * g * (l2 - l3)
    item.units = "N*m"
    add_item!(item, the_system)

    the_system

end
