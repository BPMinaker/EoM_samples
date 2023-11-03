function input_ex_bricard(; l = 0.2, m = 1.0, g = 9.81)

    the_system = mbd_system("Bricard mechanism")

    p0 = [0, 0, 0]
    p1 = [l, 0, 0]
    p2 = [l / 2, 0, -3^0.5 * l / 2]
    p3 = [0, 0, -3^0.5 * l]
    p4 = [-l / 2, 0, -3^0.5 * l / 2]
    p5 = [-l, 0, 0]

    # p1*=0.2
    # p2*=0.2
    # p3*=0.2
    # p4*=0.2
    # p5*=0.2


    item = thin_rod("Rod 1", [p0 p1], m)
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = thin_rod("Rod 2", [p1 p2], m)
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = thin_rod("Rod 3", [p2 p3], m)
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = thin_rod("Rod 4", [p3 p4], m)
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = thin_rod("Rod 5", [p4 p5], m)
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = rigid_point("Hinge 1")
    item.body[1] = "Rod 1"
    item.body[2] = "ground"
    item.location = p0
    item.forces = 3
    item.moments = 2
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    item = rigid_point("Hinge 2")
    item.body[1] = "Rod 2"
    item.body[2] = "Rod 1"
    item.location = p1
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("Hinge 3")
    item.body[1] = "Rod 3"
    item.body[2] = "Rod 2"
    item.location = p2
    item.forces = 3
    item.moments = 2
    item.axis = [cosd(30), 0, -sind(30)]
    add_item!(item, the_system)

    item = rigid_point("Hinge 4")
    item.body[1] = "Rod 4"
    item.body[2] = "Rod 3"
    item.location = p3
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("Hinge 5")
    item.body[1] = "Rod 5"
    item.body[2] = "Rod 4"
    item.location = p4
    item.forces = 3
    item.moments = 2
    item.axis = [cosd(30), 0, sind(30)]
    add_item!(item, the_system)

    item = rigid_point("Hinge 6")
    item.body[1] = "Rod 5"
    item.body[2] = "ground"
    item.location = p5
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    ## The actuator is a 'line item' and defined by two locations, location[1] attaches to body[1]...
    item = actuator("N")
    item.body[1] = "Rod 1"
    item.body[2] = "ground"
    item.location[1] = p0
    item.location[2] = p0 + [0, 0, 0.1]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    item = sensor("mglψ")
    item.body[1] = "Rod 1"
    item.body[2] = "ground"
    item.location[1] = p0
    item.location[2] = p0 + [0, 0, 0.1]
    item.twist = true
    item.gain = m * g * l
    item.units = "N*m"
    add_item!(item, the_system)

    the_system

end
