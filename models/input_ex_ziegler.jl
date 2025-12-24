function input_ex_ziegler(; P=0, m=1.0, l=1.0, k=100.0, c=0.1)

    # Ziegler's Column: A double pendulum subjected to a follower force P.
    the_system = mbd_system("Ziegler Column")

    # --------------------------------------------------------------------------
    # BODIES
    # --------------------------------------------------------------------------

    # Bar 1 (Bottom)
    # Pivot at origin (0,0,0), length l along z-axis
    # Center of mass at l/2
    item = thin_rod("bar1", [[0, 0, 0], [0, 0, l]], m; draw=true)
    add_item!(item, the_system)
    add_item!(weight(item), the_system) # Gravity? Ziegler's usually ignores gravity or includes it. Let's include weight.

    # Bar 2 (Top)
    # Pivot at end of bar1
    item = thin_rod("bar2", [[0, 0, l], [0, 0, 2l]], m; draw=true)
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    # --------------------------------------------------------------------------
    # CONSTRAINTS & JOINTS
    # --------------------------------------------------------------------------

    # Ground -> Bar 1 Hinge
    item = rigid_point("joint1")
    item.body[1] = "bar1"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 3
    item.moments = 2 # Allow rotation about y axis (planar xz)
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Rotational Spring/Damper at Joint 1
    # Joint 1 Spring/Damper
    item = flex_point("spring1")
    item.body[1] = "bar1"
    item.body[2] = "ground"
    item.location = [0, 0, 0]
    item.forces = 0 # Don't add translational stiffness (handled by joint1)
    item.moments = 1 # Apply moment about axis
    item.axis = [0, 1, 0]
    item.stiffness = [0, k] # [linear, angular]
    item.damping = [0, c]
    add_item!(item, the_system)


    # Bar 1 -> Bar 2 Hinge
    item = rigid_point("joint2")
    item.body[1] = "bar2"
    item.body[2] = "bar1"
    item.location = [0, 0, l]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # Joint 2 Spring/Damper
    item = flex_point("spring2")
    item.body[1] = "bar2"
    item.body[2] = "bar1"
    item.location = [0, 0, l]
    item.forces = 0
    item.moments = 1
    item.axis = [0, 1, 0]
    item.stiffness = [0, k]
    item.damping = [0, c]
    add_item!(item, the_system)

    # --------------------------------------------------------------------------
    # FOLLOWER FORCE
    # --------------------------------------------------------------------------

    # A follower force P stays aligned with the body frame.
    item = load("FollowerForce")
    item.body = "bar2"
    item.location = [0, 0, 2l] # Tip of bar 2
    item.force = [0, 0, -P] # Compressive force along bar2's local z-axis
    item.moment = [0, 0, 0]
    item.frame = "bar2" # Critical: Force defined in local frame
    add_item!(item, the_system)

    # --------------------------------------------------------------------------
    # SENSORS
    # --------------------------------------------------------------------------

    # Measure angles
    # Theta 1 (Joint 1)
    item = sensor("theta1")
    item.body[1] = "bar1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0.1, 0] # axis
    item.twist = 1 # angular
    item.gain = 180 / π
    item.units = "deg"
    add_item!(item, the_system)

    # Theta 2 (Joint 2 relative angle)
    item = sensor("theta2_rel")
    item.body[1] = "bar2"
    item.body[2] = "bar1"
    item.location[1] = [0, 0, l]
    item.location[2] = [0, 0.1, l]
    item.twist = 1
    item.gain = 180 / π
    item.units = "deg"
    add_item!(item, the_system)

    # Tip deflection (x)
    item = sensor("tip_x")
    item.body[1] = "bar2"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 2l]
    item.location[2] = [0.1, 0, 2l] # direction
    item.frame = 1 # Global frame
    item.units = "m"
    add_item!(item, the_system)

    # Actuator
    item = actuator("tip_x")
    item.body[1] = "bar2"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 2l]
    item.location[2] = [0.1, 0, 2l] # direction
    item.units = "N"
    add_item!(item, the_system)

    the_system
end
