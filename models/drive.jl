function drive!(the_system; a=1.2, tw=1.5, r=0.3, str = "L ", front = true)

    # Copyright (C) 2024, Bruce Minaker

    if front == true
        st = "Front "
    else
        st = "Rear "
    end

    item = body(st * "differential")
    item.location = [a, 0, r]
    add_item!(item, the_system)

    item = body(st * "diff gear")
    item.location = [a-0.1, 0, r]
    add_item!(item, the_system)

    item = rigid_point(st * "diff bearing")
    item.body[1] = st * "differential"
    item.body[2] = "Chassis"
    item.location = [a, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point(st * "diff gear bearing")
    item.body[1] = st * "diff gear"
    item.body[2] = st * "differential"
    item.location = [a-0.1, 0, r]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator(st * "axle torque")
    item.body[1] = st * "differential"
    item.body[2] = "Chassis"
    item.location[1] = [a, 0, r]
    item.location[2] = [a, -0.1, r]
    item.twist = 1
    item.units = "N*m"
    add_item!(item, the_system)

    item = sensor(st * "axle speed")
    item.body[1] = st * "differential"
    item.body[2] = "Chassis"
    item.location[1] = [a, 0, r]
    item.location[2] = [a, -0.1, r]
    item.twist = 1
    item.order = 2
    item.units = "rad/s"
    add_item!(item, the_system)


    item = body(str * "Axle")
    item.location = [a, tw/4, r]
    add_item!(item, the_system)

    item = body(str * "Stub axle")
    item.location = [a, 0.1, r]
    add_item!(item, the_system)

    item = rigid_point(str * "CV joint")
    item.body[1] = str * "Axle"
    item.body[2] = str * "Wheel+hub"
    item.location = [a, tw/2 - 0.1, r]
    item.forces = 3
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point(str * "Plunge joint")
    item.body[1] = str * "Axle"
    item.body[2] = str * "Stub axle"
    item.location = [a, 0.1, r]
    item.forces = 2
    item.moments = 1
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point(str * "Axle bearing")
    item.body[1] = str * "Stub axle"
    item.body[2] = st * "differential"
    item.location = [a, 0.1, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point(str * "Diff gear")
    item.body[1] = str * "Stub axle"
    item.body[2] = st * "diff gear"
    item.location = [a-0.1, 0.1, r]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)


end ## Leave