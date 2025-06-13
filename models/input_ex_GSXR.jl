
function input_ex_GSXR(;u)
    # Geometry points
    p2 = [1.173, 0, 0.749]
    p3 = [1.164, 0, 0.77]
    p4 = [1.342, 0, 0.426]
    p5 = [1.365, 0, 0.324]
    p6 = [1.410, 0, 0.282]
    p7 = [0.0, 0, 0.297]
    p8 = [0.6779, 0, 0.4724]
    p9 = [0.364, 0, 0.8438]
    p10 = [0.415, 0, 1.14]
    p11 = [0.549, 0, 0.3608]
    p13 = [0.487, 0, 0.4888]
    p14 = [0.196, 0, 0.3113]
    p19 = [0.539, 0, 0.1878]
    p20 = [0.4946, 0, 0.1522]
    p21 = [0.4443, 0, 0.1782]
    p22 = [0.3722, 0, 0.2748]

    # Masses
    m_ff_u = 9.99
    m_ff_l = 7.25
    m_m = 165.13
    m_rw = 14.7
    m_fw = 11.9
    m_ub = 33.68
    m_sa = 8.0

    # Inertias
    I_ff_u = [1.341, 1.548, 0.4125]
    I_m = [11.085, 22.013, 14.982]
    P_m = [0, 0, 3.691]
    I_ub = [1.428, 1.347, 0.916]
    P_ub = [0, 0, -0.443]
    I_fw = [0.27, 0.484, 0.27]
    I_rw = [0.383, 0.638, 0.383]
    I_sa = [0.02, 0.259, 0.259]

    # Stiffness and damping
    kf = 25000
    kr = 58570
    ktf = 130000
    ktr = 141000
    cr = 11650
    cf = 2134
    E = 0.4189 # caster (radians)

#    trail_b = p2[1] + p2[3] * tan(E) - p6[1]

    velocity = [u, 0, 0]

    g = 9.81

    the_system = mbd_system("Motorbike")

    # Frame
    item = body("Frame")
    item.location = p8
    item.mass = m_m
    item.moments_of_inertia = I_m
    item.products_of_inertia = P_m
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Upper body
    item = body("Upper body")
    item.location = p10
    item.mass = m_ub
    item.moments_of_inertia = I_ub
    item.products_of_inertia = P_ub
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Upper fork
    item = body("Upper fork")
    item.location = p3
    item.mass = m_ff_u
    item.moments_of_inertia = I_ff_u
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Lower fork
    item = body("Lower fork")
    item.location = p5
    item.mass = m_ff_l
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Swing arm
    item = body("Swing arm")
    item.location = p14
    item.mass = m_sa
    item.moments_of_inertia = I_sa
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Bell crank
    item = body("Bell crank")
    item.location = (p19 + p20 + p21) / 3
    item.velocity = velocity
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Front wheel
    item = body("Front wheel, bike")
    item.location = p6
    item.mass = m_fw
    item.moments_of_inertia = I_fw
    item.velocity = velocity
    item.angular_velocity = [0, u / p6[3], 0]
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Rear wheel
    item = body("Rear wheel, bike")
    item.location = p7
    item.mass = m_rw
    item.moments_of_inertia = I_rw
    item.velocity = velocity
    item.angular_velocity = [0, u / p7[3], 0]
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    # Pull rod (dogbone)
    item = link("Pull rod (dogbone)")
    item.body[1] = "Swing arm"
    item.body[2] = "Bell crank"
    item.location[1] = p22
    item.location[2] = p20
    add_item!(item, the_system)

    # Rear spring
    item = spring("Rear spring")
    item.body[1] = "Frame"
    item.body[2] = "Bell crank"
    item.location[1] = p13
    item.location[2] = p21
    item.stiffness = kr
    item.damping = cr
    add_item!(item, the_system)

    # Rear axle
    item = rigid_point("Rear axle")
    item.location = p7
    item.body[1] = "Swing arm"
    item.body[2] = "Rear wheel, bike"
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Swing arm pivot
    item = rigid_point("Swing arm pivot")
    item.location = p11
    item.body[1] = "Swing arm"
    item.body[2] = "Frame"
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Bell crank pivot
    item = rigid_point("Bell crank pivot")
    item.location = p19
    item.body[1] = "Bell crank"
    item.body[2] = "Frame"
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Seat
    item = rigid_point("Seat")
    item.location = p9
    item.body[1] = "Upper body"
    item.body[2] = "Frame"
    item.forces = 3
    item.moments = 3
    add_item!(item, the_system)

    # Steering head
    item = rigid_point("Steering head")
    item.location = p2
    item.body[1] = "Upper fork"
    item.body[2] = "Frame"
    item.forces = 3
    item.moments = 2
    item.axis = [sin(E), 0, -cos(E)]
    add_item!(item, the_system)

    # Front axle
    item = rigid_point("Front axle")
    item.location = p6
    item.body[1] = "Lower fork"
    item.body[2] = "Front wheel, bike"
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Front tire, lateral (flex point)
    item = flex_point("Front tire, lateral")
    item.location = [p6[1], 0, 0]
    item.body[1] = "Front contact patch"
    item.body[2] = "ground"
    item.damping = [20000 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Rear tire, lateral (flex point)
    item = flex_point("Rear tire, lateral")
    item.location = [0, 0, 0]
    item.body[1] = "Rear contact patch"
    item.body[2] = "ground"
    item.damping = [20000 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Front contact patch (body)
    item = body("Front contact patch")
    item.location = [p6[1], 0, 0]
    add_item!(item, the_system)

    # Rear contact patch (body)
    item = body("Rear contact patch")
    item.location = [0, 0, 0]
    add_item!(item, the_system)

    # Front contact patch constraint
    item = rigid_point("Front contact patch constraint")
    item.body[1] = "Front contact patch"
    item.body[2] = "Front wheel, bike"
    item.location = [p6[1], 0, 0]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Rear contact patch constraint
    item = rigid_point("Rear contact patch constraint")
    item.body[1] = "Rear contact patch"
    item.body[2] = "Rear wheel, bike"
    item.location = [0, 0, 0]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Front tire, sidewall
    item = rigid_point("Front tire, sidewall")
    item.body[1] = "Front wheel, bike"
    item.body[2] = "Front contact patch"
    item.location = [p6[1], 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Rear tire, sidewall
    item = rigid_point("Rear tire, sidewall")
    item.body[1] = "Rear wheel, bike"
    item.body[2] = "Rear contact patch"
    item.location = [0, 0, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Front tire, longitudinal (flex point)
    item = flex_point("Front tire, longitudinal")
    item.location = [p6[1], 0, 0]
    item.body[1] = "Front wheel, bike"
    item.body[2] = "ground"
    item.damping = [60000 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # Rear tire, longitudinal (flex point)
    item = flex_point("Rear tire, longitudinal")
    item.location = [0, 0, 0]
    item.body[1] = "Rear wheel, bike"
    item.body[2] = "ground"
    item.damping = [60000 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # Front tire, vertical
    item = flex_point("Front tire, vertical")
    item.location = [p6[1], 0, 0]
    item.body[1] = "Front wheel, bike"
    item.body[2] = "ground"
    item.stiffness = [ktf, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    # Rear tire, vertical (rigid point)
    item = flex_point("Rear tire, vertical")
    item.location = [0, 0, 0]
    item.body[1] = "Rear wheel, bike"
    item.body[2] = "ground"
    item.stiffness = [ktr, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    # Fork slider (rigid point)
    item = rigid_point("Fork slider")
    item.location = p4
    item.body[1] = "Upper fork"
    item.body[2] = "Lower fork"
    item.forces = 2
    item.moments = 3
    item.axis = p4 .- p6
    add_item!(item, the_system)

    # Right front spring
    axis = p4 .- p6
    item = spring("Right front spring")
    item.body[1] = "Upper fork"
    item.body[2] = "Lower fork"
    item.location[1] = p6 .+ 2.7 .* axis .- [0, 0.1, 0]
    item.location[2] = p6 .+ 0.2 .* axis .- [0, 0.1, 0]
    item.stiffness = kf / 2
    item.damping = cf / 2
    add_item!(item, the_system)

    # Left front spring
    item = spring("Left front spring")
    item.body[1] = "Upper fork"
    item.body[2] = "Lower fork"
    item.location[1] = p6 .+ 2.7 .* axis .+ [0, 0.1, 0]
    item.location[2] = p6 .+ 0.2 .* axis .+ [0, 0.1, 0]
    item.stiffness = kf / 2
    item.damping = cf / 2
    add_item!(item, the_system)

    item = sensor("Yaw rate")
    item.body[1] = "Frame"
    item.body[2] = "ground"
    item.location[1] = p8
    item.location[2] = p8 .- [0, 0, 0.1]
    item.order = 2
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Roll angle")
    item.body[1] = "Frame"
    item.body[2] = "ground"
    item.location[1] = p8
    item.location[2] = p8+[0.1;0;0]
    item.twist = 1
    add_item!(item, the_system);

    item = sensor("Pitch angle")
    item.body[1] = "Frame"
    item.body[2] = "ground"
    item.location[1] = p8
    item.location[2] = p8 .- [0, 0.1, 0]
    item.twist = 1
    add_item!(item, the_system)

    item = sensor("Bump")
    item.body[1] = "Frame"
    item.body[2] = "ground"
    item.location[1] = p8
    item.location[2] = p8 .- [0, 0, 0.1]
    add_item!(item, the_system)

    # Front brake actuator
    item = actuator("Front brake")
    item.body[1] = "Front wheel, bike"
    item.body[2] = "Lower fork"
    item.twist = 1
    item.location[1] = p6
    item.location[2] = p6 .+ [0, 0.1, 0]
    add_item!(item, the_system)

    # Front bump actuator
    item = actuator("Front bump")
    item.body[1] = "Front wheel, bike"
    item.body[2] = "ground"
    item.gain = ktf
    item.location[1] = [p6[1], 0, 0]
    item.location[2] = [p6[1], 0, -0.1]
    add_item!(item, the_system)

    return the_system
end