# suspension geometry is measured using an origin on the ground at the tire contact point, with x pointing forward, y pointing to the left, z up
# define the left side only, right is mirrored, so y values should all be -ve, z all +ve
# front spring geometry, lower end is fixed to the lower A-arm, and the upper end is fixed to the chassis; as the spring becomes more vertical, it becomes effectively stiffer at the wheel; similarly, as it moves outward closer to the wheel, it also becomes effectively stiffer at the wheel

# define a structure to hold the suspension geometry data
@kwdef mutable struct susp # suspension properties
    r = 0.3
    ubj = [0, -0.15, 2 * r] # upper ball joint
    lbj = [0, -0.1, r / 2] # lower ball joint
    uaapf = [0.15, -0.4, 2 * r] # upper a-arm pivot, front
    uaapr = [-0.15, -0.4, 2 * r] # upper a-arm pivot, rear
    laapf = [0.15, -0.5, r / 2] # lower a-arm pivot, front
    laapr = [0, -0.5, r / 2] # lower a-arm pivot, rear
    itre = [-0.1, -0.45, r / 2] # inner tie rod end
    otre = [-0.1, -0.12, r / 2] # outer tie rod end
    sle = [0, -0.2, r / 2] # spring lower end
    sue = [0, -0.3, 2 * r]  # spring upper end
end

# define a structure to hold the chassis parameters
@kwdef mutable struct chassis
    m = 1800.0 # mass
    hG = 0.5 # mass centre height
    Ix = 818.0 # moments of inertia
    Iy = 3267.0
    Iz = 3508.0
    a = 1.5 # wheelbase, front
    b = 1.5
    tf = 1.5
    tr = 1.5
    kf = 30000.0 # suspension stiffness, front
    kr = 30000.0
    cf = 2500.0 # suspension damping, front
    cr = 2500.0
    krf = 500.0 # anti-roll stiffness, front
    krr = 500.0
    muf = 20.0 # unsprung mass, front
    mur = 20.0
end

# full car model with steering
function input_full_car_steering(;
    u = 0.0, # speed
    kt = 180000.0, # tire vertical stiffness
    ct = 100.0, # tire vertical damping
    r = 0.3, # wheel radius
    Iw = 1.0, # wheel rotary inertia
    parms = chassis(),
    front = susp(; r),
    rear = susp(; r)
)
    # unload the chassis parameters
    (;m, hG, Ix, Iy, Iz, a, b, tf, tr, kf, kr, cf, cr, krf, krr, muf, mur) = parms

    the_system = mbd_system("EoM Sedan")
 
    item = body("Chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, Iz]
    item.location = [0, 0, hG]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    ####

    item = body("LF Wheel+hub")
    item.moments_of_inertia = [0, Iw, 0]
    item.location = [a, tf / 2, r]
    item.velocity = [u, 0, 0]
    item.angular_velocity = [0, u / r, 0]
    add_item!(item, the_system)

    item = body("LF Upright")
    item.mass = muf
    item.location = [a, tf / 2 - 0.1, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = rigid_point("LF Wheel bearing")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Upright"
    item.location = [a, tf / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LF Tie-rod")
    item.body[1] = "Rack"
    item.body[2] = "LF Upright"
    item.location[1] = front.itre + [a, tf / 2, 0]
    item.location[2] = front.otre + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = front.uaapf + [a, tf / 2, 0]
    item.location[2] = front.ubj + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Upright"
    item.location[1] = front.uaapr + [a, tf / 2, 0]
    item.location[2] = front.ubj + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = body("LF Lower A-arm")
    item.location = (front.lbj + front.laapr + front.laapf) / 3 + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Lower ball joint")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Upright"
    item.location = front.lbj + [a, tf / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LF Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = front.laapf + [a, tf / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LF Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LF Lower A-arm"
    item.location = front.laapr + [a, tf / 2, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = body("LF Anti-roll arm")
    item.location = [a - 0.25, 0.5, front.sle[3] - 0.05]
    add_item!(item, the_system)

    item = rigid_point("LF Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LF Anti-roll arm"
    item.location = [a - 0.25, 0.5, front.sle[3] - 0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LF Drop link")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "LF Anti-roll arm"
    item.location[1] = front.sle + [a, tf / 2, 0]
    item.location[2] = front.sle + [0, 0, -0.05] + [a, tf / 2, 0]
    add_item!(item, the_system)

    item = spring("LF Suspension spring")
    item.body[1] = "LF Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = front.sle + [a, tf / 2, 0]
    item.location[2] = front.sue + [a, tf / 2, 0]
    item.stiffness = kf
    item.damping = cf
    add_item!(item, the_system)

    item = spring("Front Anti-roll bar")
    item.body[1] = "LF Anti-roll arm"
    item.body[2] = "RF Anti-roll arm"
    item.location[1] = [a - 0.25, 0.45, front.sle[3] - 0.05]
    item.location[2] = item.location[1] .* [1, -1, 1]
    item.stiffness = krf
    item.preload = 0
    item.twist = 1
    add_item!(item, the_system)

    item = flex_point("LF Tire, Z")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location = [a, tf / 2, 0]
    item.stiffness = [kt, 0]
    item.damping = [ct, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LF Tire, X")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location = [a, tf / 2, 0]
    item.damping = [20000/u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("Y_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = actuator("N_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.twist = 1
    item.units = "N*m"
    item.desc = "Tire moment"
    add_item!(item, the_system)

    item = sensor("Zs_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.gain = kt
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = sensor("Zd_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2, -0.1]
    item.order = 2
    item.gain = ct
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = sensor("u_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a - 0.1, tf / 2, r]
    item.order = 2
    item.frame = 0
    item.units = "m/s"
    item.desc = "Wheel speed"
    add_item!(item, the_system)

    item = sensor("v_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, 0]
    item.location[2] = [a, tf / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.units = "rad"
    item.desc = "Tire sliding speed"
    add_item!(item, the_system)

    item = sensor("δ_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a, tf / 2, r - 0.1]
    item.twist = 1
    item.units = "rad"
    item.desc = "Steering angle"
    add_item!(item, the_system)

    item = sensor("γ_lf")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tf / 2, r]
    item.location[2] = [a - 0.1, tf / 2, r]
    item.twist = 1
    item.units = "rad"
    item.desc = "Camber angle"
    add_item!(item, the_system)


    #####

    item = body("LR Wheel+hub")
    item.moments_of_inertia = [0, Iw, 0]
    item.location = [-b, tr / 2, r]
    item.velocity = [u, 0, 0]
    item.angular_velocity = [0, u / r, 0]
    add_item!(item, the_system)

    item = body("LR Upright")
    item.mass = mur
    item.location = [-b, tr / 2 - 0.1, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = rigid_point("LR Wheel bearing")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "LR Upright"
    item.location = [-b, tr / 2, r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LR Tie-rod")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.itre + [-b, tr / 2, 0]
    item.location[2] = rear.otre + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.uaapf + [-b, tr / 2, 0]
    item.location[2] = rear.ubj + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Upright"
    item.location[1] = rear.uaapr + [-b, tr / 2, 0]
    item.location[2] = rear.ubj + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = body("LR Lower A-arm")
    item.location = (rear.lbj + rear.laapr + rear.laapf) / 3 + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = rigid_point("LR Lower ball joint")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Upright"
    item.location = rear.lbj + [-b, tr / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LR Lower A-arm pivot, front leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = rear.laapf + [-b, tr / 2, 0]
    item.forces = 3
    item.moments = 0
    add_item!(item, the_system)

    item = rigid_point("LR Lower A-arm pivot, rear leg")
    item.body[1] = "Chassis"
    item.body[2] = "LR Lower A-arm"
    item.location = rear.laapr + [-b, tr / 2, 0]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = body("LR Anti-roll arm")
    item.location = [-b + 0.25, 0.5, rear.sle[3] - 0.05]
    add_item!(item, the_system)

    item = rigid_point("LR Anti-roll arm pivot")
    item.body[1] = "Chassis"
    item.body[2] = "LR Anti-roll arm"
    item.location = [-b + 0.25, 0.5, rear.sle[3] - 0.05]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LR Drop link")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "LR Anti-roll arm"
    item.location[1] = rear.sle + [-b, tr / 2, 0]
    item.location[2] = rear.sle + [0, 0, -0.05] + [-b, tr / 2, 0]
    add_item!(item, the_system)

    item = spring("LR Suspension spring")
    item.body[1] = "LR Lower A-arm"
    item.body[2] = "Chassis"
    item.location[1] = rear.sle + [-b, tr / 2, 0]
    item.location[2] = rear.sue + [-b, tr / 2, 0]
    item.stiffness = kr
    item.damping = cr
    add_item!(item, the_system)

    item = spring("Rear anti-roll bar")
    item.body[1] = "LR Anti-roll arm"
    item.body[2] = "RR Anti-roll arm"
    item.location[1] = [-b + 0.25, 0.45, rear.sle[3] - 0.05]
    item.location[2] = item.location[1] .* [1, -1, 1]
    item.stiffness = krr
    item.preload = 0
    item.twist = 1
    add_item!(item, the_system)

    item = flex_point("LR Tire, Z")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location = [-b, tr / 2, 0]
    item.stiffness = [kt, 0]
    item.damping = [ct, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR Tire, X")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location = [-b, tr / 2, 0]
    item.damping = [20000/u, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = actuator("Y_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = actuator("N_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.twist = 1
    item.units = "N*m"
    item.desc = "Tire moment"
    add_item!(item, the_system)

    item = sensor("Zs_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.gain = kt
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = sensor("Zd_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2, -0.1]
    item.order = 2
    item.gain = ct
    item.units = "N"
    item.desc = "Tire force"
    add_item!(item, the_system)

    item = sensor("u_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b - 0.1, tr / 2, r]
    item.order = 2
    item.frame = 0
    item.units = "m/s"
    item.desc = "Wheel speed"
    add_item!(item, the_system)

    item = sensor("v_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, 0]
    item.location[2] = [-b, tr / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.units = "rad"
    item.desc = "Tire sliding speed"
    add_item!(item, the_system)

    item = sensor("δ_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "Chassis"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b, tr / 2, r - 0.1]
    item.twist = 1
    item.units = "rad"
    item.desc = "Steering angle"
    add_item!(item, the_system)

    item = sensor("γ_lr")
    item.body[1] = "LR Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [-b, tr / 2, r]
    item.location[2] = [-b - 0.1, tr / 2, r]
    item.twist = 1
    item.units = "rad"
    item.desc = "Camber angle"
    add_item!(item, the_system)


    # reflect all LF or LR items in y axis
    mirror!(the_system)
    
    item = sensor("β")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.gain = 1 / u
    item.order = 2
    item.frame = 0
    item.units = "rad"
    item.desc = "Body slip angle"
    add_item!(item, the_system)




    item = sensor("Chassis p")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.twist = 1
    item.order = 2
    item.desc = "Roll rate"
    add_item!(item, the_system)

    item = sensor("Chassis q")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.twist = 1
    item.order = 2
    item.desc = "Pitch rate"
    add_item!(item, the_system)

    item = sensor("r")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    item.order = 2
    item.units = "rad/s"
    item.desc = "Yaw rate"
    add_item!(item, the_system)

    item = sensor("y")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.desc = "Chassis lateral position"
    add_item!(item, the_system)

    item = sensor("z")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.desc = "Chassis G vertical position"
    add_item!(item, the_system)

    item = sensor("ϕ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [-0.1, 0, hG]
    item.twist = 1
    item.desc = "Chassis roll angle"
    add_item!(item, the_system)

    item = sensor("θ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, -0.1, hG]
    item.twist = 1
    item.desc = "Chassis pitch angle"
    add_item!(item, the_system)

    item = sensor("ψ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    item.desc = "Chassis yaw angle"
    add_item!(item, the_system)

    item = sensor("ru")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    item.order = 2
    item.gain = u
    item.units = "m/s^2"
    add_item!(item, the_system)

    item = sensor("α_u-δ")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, hG]
    item.location[2] = [0, 0, hG - 0.1]
    item.twist = 1
    item.order = 2
    item.gain = -(a + b) / u
    item.units = "m/s^2"
    add_item!(item, the_system)

    item = body("Rack")
    item.mass = 20
    item.location = [a + front.itre[1], 0, front.itre[3]]
    add_item!(item, the_system)

    item = rigid_point("Rack housing")
    item.body[1] = "Rack"
    item.body[2] = "Chassis"
    item.location = [a + front.itre[1], 0, front.itre[3]]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = body("Pinion")
    item.moments_of_inertia = [0.2, 0, 0]
    item.location = [a + front.itre[1]-0.1, 0, front.itre[3]+0.02]
    add_item!(item, the_system)

    item = rigid_point("Pinion housing")
    item.body[1] = "Pinion"
    item.body[2] = "Chassis"
    item.location = [a + front.itre[1], 0, front.itre[3]+0.02]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("Gear")
    item.body[1] = "Pinion"
    item.body[2] = "Rack"
    item.location = [a + front.itre[1], 0, front.itre[3] + 0.01]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = actuator("L")
    item.body[1] = "Pinion"
    item.body[2] = "Chassis"
    item.location[1] = [a + front.itre[1] - 0.2, 0, front.itre[3]+0.02]
    item.location[2] = [a + front.itre[1] - 0.3, 0, front.itre[3]+0.02]
    item.twist = 1
    item.units = "N*m"
    item.desc = "Steering torque"
    add_item!(item, the_system)

    item = flex_point("Pinion housing")
    item.body[1] = "Pinion"
    item.body[2] = "Chassis"
    item.location = [a + front.itre[1]-0.4, 0, front.itre[3]+0.02]
    item.forces = 0
    item.moments = 1
    item.axis = [1, 0, 0]
    item.stiffness = [0, 0]
    item.damping = [0, 5]
    add_item!(item, the_system)


    the_system

end

