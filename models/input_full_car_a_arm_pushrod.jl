function input_full_car_a_arm_pushrod(; u = 10.0, a = 1.2, b = 1.3, cf = 40000, cr = 40000, m = 1400, Ix = 800, Iy = 2000, Iz = 2200, r = 0.3, tw = 1.5, cs = 2000, ks = 60000)

    ## Copyright (C) 2017, Bruce Minaker
    ## full_car_a_arm_pushrod.jl is free software; you can redistribute it and/or modify it
    ## under the terms of the GNU General Public License as published by
    ## the Free Software Foundation; either version 2, or (at your option)
    ## any later version.
    ##
    ## full_car_a_arm_pushrod.jl is distributed in the hope that it will be useful, but
    ## WITHOUT ANY WARRANTY; without even the implied warranty of
    ## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    ## General Public License for more details at www.gnu.org/copyleft/gpl.html.
    ##
    ##--------------------------------------------------------------------

    the_system = mbd_system("Full Car A-Arm Pushrod")

    item = body("Chassis")
    item.mass = m
    item.moments_of_inertia = [Ix, Iy, Iz]
    item.products_of_inertia = [0.0, 0.0, 0.0]
    item.location = [0.0, 0.0, 0.5]
    item.velocity = [u, 0.0, 0.0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    susp!(the_system; str = "LF ", front = true, a, tw, r, u, cs, ks)
    tire!(the_system; str = "LF ", front = true, a, tw, cf, u)

    susp!(the_system; a = -b, str = "LR ", front = false, tw, r, u, cs, ks)
    tire!(the_system; a = -b, cf = cr, str = "LR ", front = false, tw, u)

    item = spring("Anti-roll bar")
    item.body[1] = "LF Anti-roll arm"
    item.body[2] = "RF Anti-roll arm"
    item.location[1] = [a + 0.05, 0.25, r + 0.05]
    item.location[2] = [a + 0.05, -0.25, r + 0.05]
    item.stiffness = 10.0
    item.damping = 0.0
    item.twist = 1
    add_item!(item, the_system)


    # roll centre constraints
    #=
    item=rigid_point("LF RC")
    item.body[1]="LF Wheel+hub"
    item.body[2]="ground"
    item.location=[a,tw/2,0]
    item.forces=3
    item.moments=0
    add_item!(item,the_system)

    item=rigid_point("LR RC")
    item.body[1]="LR Wheel+hub"
    item.body[2]="ground"
    item.location=[-b,tw/2,0]
    item.forces=3
    item.moments=0
    add_item!(item,the_system)
    =#

    ####% Reflect all LF or LR items in y axis
    mirror!(the_system)


    # add sensors
    item = sensor("z_LFc")
    item.body[1] = "Chassis"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("z_LFc-z_LF")
    item.body[1] = "Chassis"
    item.body[2] = "LF Wheel+hub"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.units = "m"
    add_item!(item, the_system)

    item = sensor("z_LF-u_LF")
    item.body[1] = "LF Wheel+hub"
    item.body[2] = "ground"
    item.location[1] = [a, tw / 2, r]
    item.location[2] = [a, tw / 2, r - 0.1]
    item.actuator = "u_LF "
    item.units = "m"
    add_item!(item, the_system)

    #=
    item=actuator("L")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1]=[0,0,0.5]
    item.location[2]=[0.1,0,0.5]
    item.twist=1
    item.gain=1000
    add_item!(item,the_system)

    item=sensor("ϕ")
    item.body[1]="Chassis"
    item.body[2]="ground"
    item.location[1]=[0,0,0.5]
    item.location[2]=[0.1,0,0.5]
    item.twist=1
    item.gain=180/pi
    add_item!(item,the_system)
    =#

    the_system

end ## Leave
