function input_ex_cross_axis_double_pendulum(; m=1.0, l=1.0)

    # Define the cross-axis double pendulum system
    the_system = mbd_system("Cross-Axis Double Pendulum")

    g = 9.807

    # Add the first pendulum body
    # Starts at origin, extends along -Z
    item = thin_rod("pendulum1", [[0, 0, 0], [0, 0, -l]], m; draw=true)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the second pendulum body
    # Starts at the end of the first pendulum, extends further along -Z
    item = thin_rod("pendulum2", [[0, 0, -l], [0, 0, -2l]], m; draw=true)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Joint 1: Pivots about the Y-axis at the origin
    item = rigid_point("joint1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Joint 2: Pivots about the X-axis at the end of the first pendulum
    item = rigid_point("joint2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location = [0, 0, -l]
    item.axis = [1, 0, 0]
    item.forces = 3
    item.moments = 2
    add_item!(item, the_system)

    # Actuator for Joint 1 (torque about Y)
    item = actuator("M1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    # Actuator for Joint 2 (torque about X)
    item = actuator("M2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location[1] = [0, 0, -l]
    item.location[2] = [-0.1, 0, -l]
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    # Sensors
    item = sensor("theta_1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.twist = true
    item.units = "rad"
    add_item!(item, the_system)

    item = sensor("theta_2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location[1] = [0, 0, -l]
    item.location[2] = [-0.1, 0, -l]
    item.twist = true
    item.units = "rad"
    add_item!(item, the_system)

    the_system

end
