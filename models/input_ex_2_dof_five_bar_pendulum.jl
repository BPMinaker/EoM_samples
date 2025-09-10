function input_ex_five_bar_pendulum(; m1=1.0, m2=1.5, m3=1.5, m4=1.0, k1= 100.0, k2=100.0, A=[0.0,0.0,0.0], B=[1.0,0.0,0.0], C=[-1.0,0.0,-1.0], D=[0.5,0.0,-2.0], E=[2.0,0.0,-1.0])

    # Define the system
    # Benchmark from https://www.iftomm-multibody.org

    the_system = mbd_system("Five Bar Pendulum")

    # Add the first pendulum body
    item = thin_rod("pendulum1", [A, C] , m1)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the second pendulum body
    item = thin_rod("pendulum2", [C, D] , m2)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the third pendulum body
    item = thin_rod("pendulum3", [D, E] , m3)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Add the fourth pendulum body
    item = thin_rod("pendulum4", [E, B] , m4)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # Constrain the first pendulum to rotate about the pivot point
    item = rigid_point("joint1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location = A
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("joint2")
    item.body[1] = "pendulum2"
    item.body[2] = "pendulum1"
    item.location = C
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("joint3")
    item.body[1] = "pendulum3"
    item.body[2] = "pendulum2"
    item.location = D
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("joint4")
    item.body[1] = "pendulum4"
    item.body[2] = "pendulum3"
    item.location = E
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("joint5")
    item.body[1] = "ground"
    item.body[2] = "pendulum4"
    item.location = B
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = spring("spring1")
    item.location[1] = B
    item.location[2] = C
    item.body[1] = "ground"
    item.body[2] = "pendulum1"
    item.stiffness = k1
    item.preload = 0.0
    add_item!(item, the_system)

    item = spring("spring2")
    item.location[1] = B
    item.location[2] = D
    item.body[1] = "ground"
    item.body[2] = "pendulum2"
    item.stiffness = k2
    item.preload = 0.0
    add_item!(item, the_system)

    item = load("eq")
    item.location = D
    item.body = "pendulum2"
    item.force = [0.0, 0.0, 0.5*9.81*(m1+m2+m3+m4)]
    add_item!(item, the_system)

    item = actuator("f")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = C
    item.location[2] = C - [0.1, 0, 0]
    item.units = "N"
    add_item!(item, the_system)

    item = sensor("f1")
    item.body[1] = "pendulum1"
    item.body[2] = "ground"
    item.location[1] = C
    item.location[2] = B
    item.gain = k1
    item.units = "N"
    add_item!(item, the_system)

    item = sensor("f2")
    item.body[1] = "pendulum2"
    item.body[2] = "ground"
    item.location[1] = D
    item.location[2] = B
    item.gain = k2
    item.units = "N"
    add_item!(item, the_system)

    the_system

end
