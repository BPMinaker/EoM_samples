function input_ex_pendulum_spring(; m=10, yp=-0.5, zp=0.5, ys=0.5, zs=0.5, l=0.5^0.5, θ=3π / 4, k=100, Ix=5.5)

    # a single rigid body pendulum
    the_system = mbd_system("One dof pendulum with spring")

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
    item.location = [0, yp, 1 + zp]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = spring("spring")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, ys, 1 + zs]
    item.location[2] = [0, ys + l * cos(θ), 1 + zs + l * sin(θ)]
    item.stiffness = k
    add_item!(item, the_system)

    item = actuator("Y")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [0, 1, 1]
    item.units = "N"
    add_item!(item, the_system)

    item = sensor("y")
    item.body[1] = "block"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 1]
    item.location[2] = [0, 1, 1]
    item.units = "m"
    add_item!(item, the_system)

    the_system

end
