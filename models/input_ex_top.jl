function input_ex_top(; r = 1, m = 0.01, h = 0.04, It = 0.0003, Ia = 0.0004, g = 9.81)

    # The classic spinning top problem
    # See the paper Stability analysis of rigid multibody mechanical systems with holonomic and nonholonomic constraints by M. Pappalardo et al, Arch Appl Mech (2020) 90:1961–2005

    the_system = mbd_system("Spinning Top")

    # Add the top
    item = body("top")
    item.mass = m
    item.moments_of_inertia = [It, It, Ia]
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, h]
    item.velocity = [0, 0, 0]
    item.angular_velocity = [0, 0, r]
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Add ground contact, vertical and longitudinal forces
    item = rigid_point("contact")
    item.body[1] = "top"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    # Add some inputs and outputs
    item = sensor("mghϕ")
    item.body[1] = "top"
    item.body[2] = "ground"
    item.location[1] = [0, 0, h]
    item.location[2] = [0.1, 0, h]
    item.twist = 1
    item.gain = m * g * h
    item.units = "N*m"
    item.desc = "Weight moment L"
    add_item!(item, the_system)

    item = sensor("mghθ")
    item.body[1] = "top"
    item.body[2] = "ground"
    item.location[1] = [0, 0, h]
    item.location[2] = [0, 0.1, h]
    item.twist = 1
    item.gain = m * g * h
    item.units = "N*m"
    item.desc = "Weight moment M"
    add_item!(item, the_system)

    item = actuator("L")
    item.body[1] = "top"
    item.body[2] = "ground"
    item.location[1] = [0, 0, h]
    item.location[2] = [0.1, 0, h]
    item.twist = 1
    item.units = "N*m"
    item.desc = "Applied moment L"
    add_item!(item, the_system)

    the_system

end
