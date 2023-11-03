function input_ex_pendulum(; m = 10, r = 1, Ix = 5.5 - m * r^2)

    # a single rigid body pendulum
    the_system = mbd_system("One dof pendulum")

    g = 9.807

    # add the body
    item = body("block")
    item.mass = m
    item.moments_of_inertia = [Ix, 0, 0]
    item.location = [0, 0, 1]
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = rigid_point("pin")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location = [0, 0, 1+r]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("Y")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [0, 1, 1]
    item.units = "N"
    add_item!(item, the_system)

    item = actuator("L")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [1, 0, 1]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    item = sensor("y")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [0, 1, 1]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("mgrϕ")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [1, 0, 1]
    item.twist = true
    item.gain = m * g * r
    item.units = "N*m"
    add_item!(item, the_system)


    the_system

end
