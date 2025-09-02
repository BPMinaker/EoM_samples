function input_double_pendulum(; m=1.0, l=1.0, x=0)

    draw = any(string.(Base.loaded_modules_array()) .== "EoM_X3D")
    if draw
        println("Drawing...")
    end

    # Define the double pendulum system
    the_system = mbd_system("Double Pendulum")

    g = 9.807

    # Add the first pendulum body
    item = thin_rod("pendulum1", [[0, 0, x * l], [0, 0, -(1 - x) * l]], m)
    if draw
        item.x3d = EoM_X3D.x3d_cyl([[0, 0, l / 2] [0, 0, -l/2]], rad=0.018, col=[0.25, 0.25, 0.25])
    end
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the second pendulum body
    item = thin_rod("pendulum2", [[0, 0.05, -(1 - 2x) * l], [0, 0.05, -(1 - x) * 2l]], m)
    if draw
        item.x3d = EoM_X3D.x3d_cyl([[0, 0, l / 2] [0, 0, -l/2]], rad=0.018, col=[0.25, 0.25, 0.25])
    end
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
    item.location = [0, 0, -(1 - x) * l]
    item.axis = [0, 1, 0]
    item.forces = 3
    item.moments = 2
    add_item!(item, the_system)

    item = actuator("M1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    item = actuator("M2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location[1] = [0, 0, -(1 - x) * l]
    item.location[2] = [0, -0.1, -(1 - x) * l]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    # Define sensors for angles and angular velocities
    item = sensor("mglθ1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.twist = true
    item.gain = m * g * l
    item.units = "N*m"
    add_item!(item, the_system)

    # Note relative coordinate output
    item = sensor("mgl(θ2-θ1)")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location[1] = [0, 0, -(1 - x) * l]
    item.location[2] = [0, -0.1, -(1 - x) * l]
    item.twist = true
    item.gain = m * g * l
    item.units = "N*m"
    add_item!(item, the_system)

    the_system

end
