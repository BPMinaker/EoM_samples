function input_ex_b_train(;
    u=20,
    # Tractor parameters
    a=1.289,             # Front axle to CG
    b=2.885 - 1.289,     # Rear axle to CG
    d1=2.7,              # Tractor CG to Fifth Wheel 1
    m1=7000,             # Tractor Mass
    Iz1=3500,            # Tractor Yaw Inertia
    cf=80000,            # Front Axle Cornering Stiffness
    cr=80000,            # Rear Axle Cornering Stiffness

    # Lead Trailer parameters
    e1=2.7,              # Fifth Wheel 1 to Lead Trailer CG
    h1=3.0,              # Lead Trailer CG to Lead Trailer Axle
    d2=4.0,              # Lead Trailer CG to Fifth Wheel 2 (Rear of lead trailer)
    m2=8000,             # Lead Trailer Mass
    Iz2=4000,            # Lead Trailer Yaw Inertia
    c2=80000,            # Lead Trailer Axle Cornering Stiffness

    # Pup Trailer parameters
    e2=3.0,              # Fifth Wheel 2 to Pup Trailer CG
    h2=3.0,              # Pup Trailer CG to Pup Trailer Axle
    m3=8000,             # Pup Trailer Mass
    Iz3=4000,            # Pup Trailer Yaw Inertia
    c3=80000             # Pup Trailer Axle Cornering Stiffness
)

    the_system = mbd_system("B-Train Tractor Trailer")

    if (u == 0)
        error("Speed must be non-zero.")
    end

    # --------------------------------------------------------------------------
    # BODIES
    # --------------------------------------------------------------------------

    # 1. Tractor
    item = body("tractor")
    item.mass = m1
    item.moments_of_inertia = [0, 0, Iz1]
    item.location = [0, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    # 2. Lead Trailer
    # Location: Tractor CG (0) - d1 (to hitch) - e1 (to lead CG)
    lead_cg_x = -d1 - e1
    item = body("lead_trailer")
    item.mass = m2
    item.moments_of_inertia = [0, 0, Iz2]
    item.location = [lead_cg_x, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    # 3. Pup Trailer
    # Location: Lead CG - d2 (to hitch 2) - e2 (to pup CG)
    pup_cg_x = lead_cg_x - d2 - e2
    item = body("pup_trailer")
    item.mass = m3
    item.moments_of_inertia = [0, 0, Iz3]
    item.location = [pup_cg_x, 0, 0]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)

    # --------------------------------------------------------------------------
    # TIRES (Flex Points)
    # --------------------------------------------------------------------------

    # Tractor Front Tire
    item = flex_point("tractor front tire")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location = [a, 0, 0]
    item.damping = [cf / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Tractor Rear Tire
    item = flex_point("tractor rear tire")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location = [-b, 0, 0]
    item.damping = [cr / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Lead Trailer Tire
    item = flex_point("lead trailer tire")
    item.body[1] = "lead_trailer"
    item.body[2] = "ground"
    item.location = [lead_cg_x - h1, 0, 0]
    item.damping = [c2 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Pup Trailer Tire
    item = flex_point("pup trailer tire")
    item.body[1] = "pup_trailer"
    item.body[2] = "ground"
    item.location = [pup_cg_x - h2, 0, 0]
    item.damping = [c3 / u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # --------------------------------------------------------------------------
    # CONSTRAINTS (Rigid Points)
    # --------------------------------------------------------------------------

    # 1. Road Constraint (Keep tractor on ground plane)
    item = rigid_point("road")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location = [0, 0, 0] # At tractor CG
    item.forces = 1 # Constrain z
    item.moments = 2 # Constrain roll and pitch
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # 2. Hitch 1 (Fifth Wheel: Tractor <-> Lead Trailer)
    item = rigid_point("hitch1")
    item.body[1] = "tractor"
    item.body[2] = "lead_trailer"
    item.location = [-d1, 0, 0]
    item.forces = 3 # Constrain x, y, z
    item.moments = 2 # Constrain roll and pitch (allow yaw)
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # 3. Hitch 2 (Fifth Wheel: Lead Trailer <-> Pup Trailer)
    # Location is at Lead CG - d2
    hitch2_x = lead_cg_x - d2
    item = rigid_point("hitch2")
    item.body[1] = "lead_trailer"
    item.body[2] = "pup_trailer"
    item.location = [hitch2_x, 0, 0]
    item.forces = 3 # Constrain x, y, z
    item.moments = 2 # Constrain roll and pitch (allow yaw)
    item.axis = [0, 0, 1]
    add_item!(item, the_system)

    # 4. Forward Speed Constraint
    item = rigid_point("speed")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 1 # Constrain x motion
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # --------------------------------------------------------------------------
    # INPUTS & SENSORS
    # --------------------------------------------------------------------------

    # Steer Input
    item = actuator("δ_f")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location[1] = [a, 0, 0]
    item.location[2] = [a, 0.1, 0]
    item.gain = cf * π / 180
    item.units = "°"
    item.desc = "Steer angle"
    add_item!(item, the_system)

    # Tractor Yaw Rate
    item = sensor("r1")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, 0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = 180 / π
    item.units = "°/s"
    item.desc = "Tractor Yaw Rate"
    add_item!(item, the_system)

    # Tractor Sideslip
    item = sensor("β")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u
    item.units = "°"
    item.desc = "Tractor Sideslip"
    add_item!(item, the_system)

    # Articulation Angle 1 (Tractor - Lead)
    item = sensor("γ1")
    item.body[1] = "tractor"
    item.body[2] = "lead_trailer"
    item.location[1] = [-d1, 0, 0]
    item.location[2] = [-d1, 0, 0.1]
    item.twist = 1 # angular
    item.gain = 180 / π
    item.units = "°"
    item.desc = "Lead Trailer Articulation"
    add_item!(item, the_system)

    # Articulation Angle 2 (Lead - Pup)
    item = sensor("γ2")
    item.body[1] = "lead_trailer"
    item.body[2] = "pup_trailer"
    item.location[1] = [hitch2_x, 0, 0]
    item.location[2] = [hitch2_x, 0, 0.1]
    item.twist = 1 # angular
    item.gain = 180 / π
    item.units = "°"
    item.desc = "Pup Trailer Articulation"
    add_item!(item, the_system)

    # Lateral Acceleration (Tractor)
    item = sensor("ay1")
    item.body[1] = "tractor"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0]
    item.order = 3 # acceleration
    item.gain = 1 / 9.81
    item.units = "g"
    item.desc = "Tractor Lat Accel"
    add_item!(item, the_system)

    the_system

end
