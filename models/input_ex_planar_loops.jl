function input_ex_planar_loops()

## Copyright (C) 2017, Bruce Minaker
## input_ex_smd.jl is free software; you can redistribute it and/or modify it
## under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2, or (at your option)
## any later vrsion.
##
## input_ex_smd.jl is distributed in the hope that it will be useful, but
## WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
## General Public License for more details at www.gnu.org/copyleft/gpl.html.
##
##--------------------------------------------------------------------

    the_system = mbd_system("Planar loops")

    item = body("block 1")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.2,0,-0.2]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 2")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.2,0,-0.2]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 3")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.4,0,-0.4]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 4")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0,0,-0.4]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 5")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.4,0,-0.4]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)


    item = body("block 6")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.2,0,-0.6]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 7")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.2,0,-0.6]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 8")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.6,0,-0.6]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 9")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.4,0,-0.8]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)


    item = body("block 10")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.6,0,-0.6]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 11")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.4,0,-0.8]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)



    item = body("block 12")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.4,0,0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 13")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [-0.6,0,-0.2]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)


    item = body("block 14")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.4,0,0]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)

    item = body("block 15")
    item.mass = 1.0
    item.moments_of_inertia = [0.1,0.1,0.1]
    item.location = [0.6,0,-0.2]
    add_item!(item, the_system)
    add_item!(weight(item), the_system)


    item = rigid_point("pin 1")
    item.body[1] = "block 1"
    item.body[2] = "ground"
    item.location = [-0.1,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 2")
    item.body[1] = "block 2"
    item.body[2] = "ground"
    item.location = [0.1,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 3")
    item.body[1] = "block 1"
    item.body[2] = "block 3"
    item.location = [-0.3,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 4")
    item.body[1] = "block 1"
    item.body[2] = "block 4"
    item.location = [-0.1,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 5")
    item.body[1] = "block 2"
    item.body[2] = "block 4"
    item.location = [0.1,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 6")
    item.body[1] = "block 2"
    item.body[2] = "block 5"
    item.location = [0.3,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)


    item = rigid_point("pin 7")
    item.body[1] = "block 3"
    item.body[2] = "block 6"
    item.location = [-0.3,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 8")
    item.body[1] = "block 4"
    item.body[2] = "block 6"
    item.location = [-0.1,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 9")
    item.body[1] = "block 4"
    item.body[2] = "block 7"
    item.location = [0.1,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 10")
    item.body[1] = "block 5"
    item.body[2] = "block 7"
    item.location = [0.3,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)


    item = rigid_point("pin 11")
    item.body[1] = "block 3"
    item.body[2] = "block 8"
    item.location = [-0.5,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 12")
    item.body[1] = "block 8"
    item.body[2] = "block 9"
    item.location = [-0.5,0,-0.7]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 13")
    item.body[1] = "block 6"
    item.body[2] = "block 9"
    item.location = [-0.3,0,-0.7]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 14")
    item.body[1] = "block 5"
    item.body[2] = "block 10"
    item.location = [0.5,0,-0.5]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 15")
    item.body[1] = "block 10"
    item.body[2] = "block 11"
    item.location = [0.5,0,-0.7]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 16")
    item.body[1] = "block 11"
    item.body[2] = "block 7"
    item.location = [0.3,0,-0.7]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)


    item = rigid_point("pin 17")
    item.body[1] = "block 3"
    item.body[2] = "block 13"
    item.location = [-0.5,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 18")
    item.body[1] = "block 12"
    item.body[2] = "block 13"
    item.location = [-0.5,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 19")
    item.body[1] = "block 1"
    item.body[2] = "block 12"
    item.location = [-0.3,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 20")
    item.body[1] = "block 5"
    item.body[2] = "block 15"
    item.location = [0.5,0,-0.3]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 21")
    item.body[1] = "block 14"
    item.body[2] = "block 15"
    item.location = [0.5,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)

    item = rigid_point("pin 22")
    item.body[1] = "block 2"
    item.body[2] = "block 14"
    item.location = [0.3,0,-0.1]
    item.forces=3
    item.moments=2
    item.axis=[0,1,0]
    add_item!(item, the_system)



    item = actuator("X")
    item.body[1] = "block 3"
    item.body[2] = "ground"
    item.location[1] = [0,0,-0.4]
    item.location[2] = [-0.1,0,-0.4]
    add_item!(item, the_system)

    item = sensor("kx")
    item.body[1] = "block 3"
    item.body[2] = "ground"
    item.location[1] = [0,0,-0.4]
    item.location[2] = [-0.1,0,-0.4]
    item.gain = 9.81
    item.units = "N"
    add_item!(item, the_system)

    the_system
end
