
# define struct to hold all the parameters and their default values, so we don't have to list them all in the function definition
@kwdef mutable struct params_list
    u = 10
    a = 1.2
    cf = 40000
    m = 400
    mu = 40
    r = 0.3
    tw = 1.5
    ks = 80000
    cs = 4000
    kt = 150000
    ct = 100
    hui = 0.49
    huo = 0.49
    hli = 0.15
    hlo = 0.15
    wui = 0.43
    wuo = 0.745
    wli = 0.43
    wlo = 0.745
    Y = 1000
    g = 9.81
end

function input_quarter_car_planar_a_arm(; kwargs...)

    (; u, a, cf, m, mu, r, tw, ks, cs, kt, ct, hui, huo, hli, hlo, wui, wuo, wli, wlo, Y, g) = params_list(; kwargs...)

    the_system = mbd_system("Quarter Car A-Arm")
    the_system.scratch = kinematics(; r_A=[wui, hui], r_B=[wuo, huo], r_C=[wli, hli], r_D=[wlo, hlo], r_E=[tw/2, 0])

    item = body("Chassis")
    item.mass = m
    item.moments_of_inertia = [0, 0, 0]
    item.products_of_inertia = [0, 0, 0]
    item.location = [0, 0, 0.3]
    add_item!(item, the_system)
    add_item!(weight(item, g), the_system)

    item = rigid_point("Chassis constraint")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location = [0, 0, 0.3]
    item.forces = 2
    item.moments = 3
    item.axis = [0, 0, 1]
    add_item!(item, the_system)


    item = body("LF Wheel+hub")
    item.mass = mu
    item.location = [a, tw / 2, r]
    item.velocity = [u, 0, 0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("LF Axle")
    item.location = [a, tw / 2 - 0.15, r]
    add_item!(item, the_system)

    item=thin_rod("LF Lower arm", [[a,wli,hli], [a,wlo,hlo]], 0)
    add_item!(item, the_system)

    item=thin_rod("LF Upper arm", [[a,wui,hui], [a,wuo,huo]], 0)
    add_item!(item, the_system)


    item = rigid_point("LF Wheel bearing")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "LF Axle"
    item.location = [a, tw / 2, r]
    item.forces = 3
    item.moments = 3
#    item.axis = [0, 1, 0]
    add_item!(item, the_system)


    # suspension constraints
    item = rigid_point("LF Upper A-Arm pivot")
    item.body[1] = "LF Upper arm"
    item.body[2] = "Chassis"
    item.location = [a, wui, hui]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Lower A-Arm pivot")
    item.body[1] = "LF Lower arm"
    item.body[2] = "Chassis"
    item.location = [a, wli, hli]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)


    item = rigid_point("LF Upper ball joint")
    item.body[1] = "LF Upper arm"
    item.body[2] = "LF Axle"
    item.location = [a, wuo, huo]
    item.forces = 3
    item.moments = 2
    item.axis = [1, 0, 0]
    add_item!(item, the_system)

    item = rigid_point("LF Lower ball joint")
    item.body[1] = "LF Lower arm"
    item.body[2] = "LF Axle"
    item.location = [a, wlo, hlo]
    item.forces = 2
    item.moments = 0
    item.axis = [1, 0, 0]
    add_item!(item, the_system)


    # front suspension
    item = spring("LF Spring")
    item.body[1] = "LF Lower arm"
    item.body[2] = "Chassis"
    item.location[1] = [a, 0.5*(wlo+wli), 0.5*(hlo+hli)]
    item.location[2] = [a, 0.5*(wlo+wli), 0.5*(hlo+hli)+0.4]
    item.stiffness = ks
    item.damping = cs
    add_item!(item, the_system)


    item = load("Y")
    item.body = ("LF Wheel+hub")
    item.location = [a, tw / 2, 0]
    item.force = [0, Y, 0]
    add_item!(item, the_system)

    tire!(the_system; a, tw, kt, ct, cf, u, str = "LF ")

    # add sensors
    item = sensor("LF Chassis motion")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("LF Suspension travel")
    item.body[1] = "Chassis"
    item.body[2] = "LF Wheel+hub"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("LF Tire compression")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.actuator = "LF Bump"
    item.units = "m"
    add_item!(item, the_system)

    item = actuator("LF Bump")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, 0]
    item.location[2] = [a, tw / 2, -0.1]
    item.gain = kt
    item.rate_gain = ct
    item.units = "m"
    add_item!(item, the_system)

    the_system

end ## Leave
