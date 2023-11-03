function tire!(the_system; a = 1.2, tw = 1.5, kt = 150000, ct = 100, cf = 30000, u = 10.0, g = 9.81, str = "L ", front = true)

    # Copyright (C) 2019, Bruce Minaker

    velocity = [u, 0, 0]

    # Tires (flex_points)

    item = flex_point(str * "Tire, vertical")
    item.body[1] = str * "Wheel+hub"
    item.body[2] = "ground"
    item.location = [a, tw / 2, 0]
    item.stiffness = [kt, 0]
    item.damping = [ct, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    item = body(str * "Contact patch")
    item.location = [a, tw / 2, 0]
    add_item!(item, the_system)

    item = rigid_point(str * "Contact patch constraint")
    item.body[1] = str * "Contact patch"
    item.body[2] = str * "Wheel+hub"
    item.location = [a, tw / 2, 0]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point(str * "Tire, sidewall")
    item.body[1] = str * "Wheel+hub"
    item.body[2] = str * "Contact patch"
    item.location = [a, tw / 2, 0]
    item.stiffness = [135000, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point(str * "Tire, horizontal")
    item.body[1] = str * "Contact patch"
    item.body[2] = "ground"
    item.location = [a, tw / 2, 0]
    item.damping = [cf / u, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

end ## Leave