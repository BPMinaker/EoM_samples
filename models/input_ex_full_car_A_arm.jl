# define a structure to hold the suspension geometry data
mutable struct susp
    r::Float64 # wheel radius
    ubj::Vector{Float64} # upper ball joint
    lbj::Vector{Float64} # lower ball joint
    uaapf::Vector{Float64} # upper a-arm pivot, front
    uaapr::Vector{Float64} # upper a-arm pivot, rear
    laapf::Vector{Float64} # lower a-arm pivot, front
    laapr::Vector{Float64} # lower a-arm pivot, rear
    itre::Vector{Float64} # inner tie rod end
    otre::Vector{Float64} # outer tie rod end
    sle::Vector{Float64} # spring lower end
    sue::Vector{Float64} # spring upper end
end
# define a constructor for the susp structure that adds keyword arguments
susp(;r, ubj, lbj, uaapf, uaapr, laapf, laapr, itre, otre, sle, sue) =
susp(r, ubj, lbj, uaapf, uaapr, laapf, laapr, itre, otre, sle, sue)

# define a structure to hold all vehicle parameters
mutable struct list
    u::Float64 # speed
    m::Float64 # mass
    a::Float64 # wheelbase, front
    b::Float64
    tf::Float64 # track width, front
    tr::Float64
    hG::Float64 # mass centre height
    Ix::Float64 # moments of inertia
    Iy::Float64
    Iz::Float64
    kf::Float64 # suspension stiffness, front
    kr::Float64
    cf::Float64 # suspension damping, front
    cr::Float64
    krf::Float64 # anti-roll stiffness, front
    krr::Float64
    muf::Float64 # unsprung mass, front
    mur::Float64
    cfy::Float64 # cornering stiffness, front
    cry::Float64
    kt::Float64 # tire vertical stiffness
    gear::Vector{Float64} # gear ratios
    fd::Float64 # final drive ratio
end
# define a constructor for the list structure that adds keyword arguments
list(;u, m, a, b, tf, tr, hG, Ix, Iy, Iz, kf, kr, cf, cr, krf, krr, muf, mur, cfy, cry, kt, gear, fd) =
list(u, m, a, b, tf, tr, hG, Ix, Iy, Iz, kf, kr, cf, cr, krf, krr, muf, mur, cfy, cry, kt, gear, fd)

function input_full_car_a_arm(;params::list, front::susp, rear::susp)

    the_system = mbd_system("Full Car Model with A-arms")

    # add one body representing the chassis
    item = body("chassis")
    item.mass = params.m
    item.moments_of_inertia = [params.Ix, params.Iy, params.Iz]
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, params.hG]
    item.velocity = [params.u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF wheel")
    item.mass = params.muf
    item.location = [params.a, params.tf / 2, front.r]
    item.velocity = [params.u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LR wheel")
    item.mass = params.mur
    item.location = [-params.b, params.tr / 2, rear.r]
    item.velocity = [params.u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF axle")
    item.mass = 0
    item.location = [params.a, params.tf / 2 - 0.15, front.r]
    item.velocity = [params.u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = link("LF Tie-rod")
    item.body[1] = "chassis"
    item.body[2] = "LF axle"
    item.location[1] = front.itre + [params.a, params.tf / 2, 0]
    item.location[2] = front.otre + [params.a, params.tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, front leg")
    item.body[1] = "chassis"
    item.body[2] = "LF axle"
    item.location[1] = front.uaapf + [params.a, params.tf / 2, 0]
    item.location[2] = front.ubj + [params.a, params.tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Upper A-arm, rear leg")
    item.body[1] = "chassis"
    item.body[2] = "LF axle"
    item.location[1] = front.uaapr + [params.a, params.tf / 2, 0]
    item.location[2] = front.ubj + [params.a, params.tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Lower A-arm, front leg")
    item.body[1] = "chassis"
    item.body[2] = "LF axle"
    item.location[1] = front.laapf + [params.a, params.tf / 2, 0]
    item.location[2] = front.lbj + [params.a, params.tf / 2, 0]
    add_item!(item, the_system)

    item = link("LF Lower A-arm, rear leg")
    item.body[1] = "chassis"
    item.body[2] = "LF axle"
    item.location[1] = front.laapr + [params.a, params.tf / 2, 0]
    item.location[2] = front.lbj + [params.a, params.tf / 2, 0]
    add_item!(item, the_system)

    item = body("LR axle")
    item.mass = 0
    item.location = [-params.b, params.tr / 2 - 0.15, rear.r]
    item.velocity = [params.u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = link("LR Tie-rod")
    item.body[1] = "chassis"
    item.body[2] = "LR axle"
    item.location[1] = rear.itre + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.otre + [-params.b, params.tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, front leg")
    item.body[1] = "chassis"
    item.body[2] = "LR axle"
    item.location[1] = rear.uaapf + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.ubj + [-params.b, params.tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Upper A-arm, rear leg")
    item.body[1] = "chassis"
    item.body[2] = "LR axle"
    item.location[1] = rear.uaapr + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.ubj + [-params.b, params.tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Lower A-arm, front leg")
    item.body[1] = "chassis"
    item.body[2] = "LR axle"
    item.location[1] = rear.laapf + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.lbj + [-params.b, params.tr / 2, 0]
    add_item!(item, the_system)

    item = link("LR Lower A-arm, rear leg")
    item.body[1] = "chassis"
    item.body[2] = "LR axle"
    item.location[1] = rear.laapr + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.lbj + [-params.b, params.tr / 2, 0]
    add_item!(item, the_system)

    ###############

    item = rigid_point("LF wheel bearing")
    item.body[1] = "LF wheel"
    item.body[2] = "LF axle"
    item.location = [params.a, params.tf / 2, front.r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR wheel bearing")
    item.body[1] = "LR wheel"
    item.body[2] = "LR axle"
    item.location = [-params.b, params.tr / 2, rear.r]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LF wheel, X")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location = [params.a, params.tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LR wheel, X")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location = [-params.b, params.tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    # anti-roll
    item = body("LF anti-roll arm")
    item.location = [params.a - 0.2, params.tf / 2 - front.r / 2, front.r - 0.1]
    add_item!(item, the_system)

    item = body("LR anti-roll arm")
    item.location = [-params.b + 0.2, params.tr / 2 - rear.r / 2, rear.r - 0.1]
    add_item!(item, the_system)

    item = rigid_point("LF anti-roll hinge")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "chassis"
    item.location = [params.a - 0.2, params.tf / 2 - front.r / 2, front.r - 0.1]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = rigid_point("LR anti-roll hinge")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "chassis"
    item.location = [-params.b + 0.2, params.tr / 2 - rear.r / 2, rear.r - 0.1]
    item.forces = 3
    item.moments = 2
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = link("LF drop link")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "LF axle"
    item.location[1] = [params.a, params.tf / 2 - front.r / 2, front.r - 0.1]
    item.location[2] = [params.a, params.tf / 2 - front.r / 2, front.r]
    add_item!(item, the_system)

    item = link("LR drop link")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "LR axle"
    item.location[1] = [-params.b, params.tr / 2 - rear.r / 2, rear.r - 0.1]
    item.location[2] = [-params.b, params.tr / 2 - rear.r / 2, rear.r]
    add_item!(item, the_system)

    # anti-roll bars
    # note that the right side bodies will come from the mirror
    item = spring("F anti-roll")
    item.body[1] = "LF anti-roll arm"
    item.body[2] = "RF anti-roll arm"
    item.location[1] = [params.a - 0.2, params.tf / 2 - 0.2, front.r - 0.1]
    item.location[2] = [params.a - 0.2, -params.tf / 2 + 0.2, front.r - 0.1]
    item.stiffness = params.krf
    item.twist = 1
    item.preload = 0
    add_item!(item, the_system)

    item = spring("R anti-roll")
    item.body[1] = "LR anti-roll arm"
    item.body[2] = "RR anti-roll arm"
    item.location[1] = [-params.b + 0.2, params.tr / 2 - 0.2, rear.r - 0.1]
    item.location[2] = [-params.b + 0.2, -params.tr / 2 + 0.2, rear.r - 0.1]
    item.stiffness = params.krr
    item.twist = 1
    item.preload = 0
    add_item!(item, the_system)

    # front suspension
    item = spring("LF spring")
    item.body[1] = "LF axle"
    item.body[2] = "chassis"
    item.location[1] = front.sle + [params.a, params.tf / 2, 0]
    item.location[2] = front.sue + [params.a, params.tf / 2, 0]
    item.stiffness = params.kf
    item.damping = params.cf
    add_item!(item, the_system)

    # rear suspension
    item = spring("LR spring")
    item.body[1] = "LR axle"
    item.body[2] = "chassis"
    item.location[1] = rear.sle + [-params.b, params.tr / 2, 0]
    item.location[2] = rear.sue + [-params.b, params.tr / 2, 0]
    item.stiffness = params.kr
    item.damping = params.cr
    add_item!(item, the_system)

    # tire vertical stifness
    item = flex_point("LF tire, Z")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.stiffness = [params.kt, 0]
    item.damping = [0, 0]
    item.location = [params.a, params.tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR tire, Z")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.stiffness = [params.kt, 0]
    item.damping = [0, 0]
    item.location = [-params.b, params.tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 0, 1]
    item.rolling_axis = [0, 1, 0]
    add_item!(item, the_system)

    # tire cornering stiffness
    item = flex_point("LF tire, Y")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.damping = [params.cfy / params.u, 0]
    item.location = [params.a, params.tf / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    item = flex_point("LR tire, Y")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.damping = [params.cry / params.u, 0]
    item.location = [-params.b, params.tr / 2, 0]
    item.forces = 1
    item.moments = 0
    item.axis = [0, 1, 0]
    add_item!(item, the_system)

    # tire lateral force
    item = actuator("Y_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location[1] = [params.a, params.tf / 2, 0]
    item.location[2] = [params.a, params.tf / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire lateral force"
    add_item!(item, the_system)

    item = actuator("Y_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location[1] = [-params.b, params.tr / 2, 0]
    item.location[2] = [-params.b, params.tr / 2 - 0.1, 0]
    item.units = "N"
    item.desc = "Tire lateral force"
    add_item!(item, the_system)

    item = actuator("u_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.gain = params.kt
    item.location[1] = [params.a, params.tf / 2, 0.1]
    item.location[2] = [params.a, params.tf / 2, 0]
    item.units = "m"
    item.desc = "Road bump"
    add_item!(item, the_system)

    item = actuator("u_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.gain = params.kt
    item.location[1] = [-params.b, params.tr / 2, 0.1]
    item.location[2] = [-params.b, params.tr / 2, 0]
    item.units = "m"
    item.desc = "Road bump"
    add_item!(item, the_system)

    # tire measure vertical force
    item = sensor("Z_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.gain = params.kt
    item.location[1] = [params.a, params.tf / 2, 0.1]
    item.location[2] = [params.a, params.tf / 2, 0]
    item.actuator = "u_lf"
    item.actuator_gain = params.kt
    item.units = "N"
    item.desc = "Tire vertical force"
    add_item!(item, the_system)

    item = sensor("Z_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.gain = params.kt
    item.location[1] = [-params.b, params.tr / 2, 0.1]
    item.location[2] = [-params.b, params.tr / 2, 0]
    item.actuator = "u_lr"
    item.actuator_gain = params.kt
    item.units = "N"
    item.desc = "Tire vertical force"
    add_item!(item, the_system)

    # tire, measure slip angle
    item = sensor("α_lf")
    item.body[1] = "LF wheel"
    item.body[2] = "ground"
    item.location[1] = [params.a, params.tf / 2, 0]
    item.location[2] = [params.a, params.tf / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.gain = 1 / params.u
    item.units = "rad"
    item.desc = "Tire slip angle"
    add_item!(item, the_system)

    item = sensor("α_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "ground"
    item.location[1] = [-params.b, params.tr / 2, 0]
    item.location[2] = [-params.b, params.tr / 2 - 0.1, 0]
    item.order = 2
    item.frame = 0
    item.gain = 1 / params.u
    item.units = "rad"
    item.desc = "Tire slip angle"
    add_item!(item, the_system)

    item = actuator("m_lr")
    item.body[1] = "LR wheel"
    item.body[2] = "chassis"
    item.location[1] = [-params.b, params.tr / 2, rear.r]
    item.location[2] = [-params.b, params.tr / 2 - 0.1, rear.r]
    item.twist = 1
    item.units = "N*m"
    item.desc = "LR Axle torque"
    add_item!(item, the_system)

    mirror!(the_system) # note that the mirror can't go any further without adressing the change in the location in the sequence of items in the main file

    # 13
    item = sensor("r")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, 0, 0]
    item.twist = 1
    item.order = 2
    item.gain = 180 / π
    item.units = "°/s"
    item.desc = "Yaw rate"
    add_item!(item, the_system)

    # measure the bounce, pitch, and roll
    # 14
    item = sensor("z_G")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, 0, 0]
    item.gain = 1000
    item.units = "mm"
    item.desc = "Vertical displacement"
    add_item!(item, the_system)

    #15
    item = sensor("ϕ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [-0.1, 0, 0.25]
    item.gain = 180 / π
    item.twist = 1
    item.units = "°"
    item.desc = "Roll angle"
    add_item!(item, the_system)

    #16
    item = sensor("θ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, -0.1, 0.25]
    item.gain = 180 / π
    item.twist = 1
    item.units = "°"
    item.desc = "Pitch angle"
    add_item!(item, the_system)

    #17
    item = sensor("β")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0.25]
    item.location[2] = [0, -0.1, 0.25]
    item.order = 2 # velocity
    item.frame = 0 # local frame
    item.gain = 180 / π / u # radian to degree
    item.units = "°"
    item.desc = "Slip angle"
    add_item!(item, the_system)

    #18
    item = sensor("α_u-δ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, params.hG]
    item.location[2] = [0, 0, -0.1]
    item.twist = 1 # angular
    item.order = 2 # velocity
    item.gain = -180 * (params.a + params.b) / π / params.u # radian to degree
    item.units = "°"
    item.desc = "Understeer term"
    add_item!(item, the_system)

    #19
    item = sensor("zddot_P")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [params.a / 2, params.tf / 4, params.hG]
    item.location[2] = [params.a / 2, params.tf / 4, params.hG - 0.1]
    item.order = 3 # acceleration
    item.units = "m/s^2"
    item.desc = "Vertical acceleration"
    add_item!(item, the_system)

    item = sensor("x_G")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, params.hG]
    item.location[2] = [-0.1, 0, params.hG]
    item.units = "m"
    item.desc = "Longitudinal position"
    add_item!(item, the_system)

    item = sensor("u_G")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, params.hG]
    item.location[2] = [-0.1, 0, params.hG]
    item.order = 2
    item.gain = 3.6
    item.units = "km/hr"
    item.desc = "Velocity"
    add_item!(item, the_system)

    item = actuator("Xa")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, params.hG]
    item.location[2] = [0.1, 0, params.hG]
    item.units = "N"
    item.desc = "Aerodynamic force"
    add_item!(item, the_system)

    # note that the y location will not reach steady state with constant delta input, so adding the sensor will give an error if the steady state gain is computed, but is included so that a time history can be computed
    item = sensor("y")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, -0.1, 0]
    item.units = "m"
    item.desc = "Lateral position"
    add_item!(item, the_system)

    # also won't reach steady state with constant delta input
    item = sensor("ψ")
    item.body[1] = "chassis"
    item.body[2] = "ground"
    item.location[1] = [0, 0, 0]
    item.location[2] = [0, 0, -0.1]
    item.twist = 1 # angular
    item.gain = 180 / π
    item.units = "°"
    item.desc = "Yaw angle"
    add_item!(item, the_system)

    the_system

end

