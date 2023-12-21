function input_ex_hanging_chain(; nbodys = 15)

    ## Copyright (C) 2017, Bruce Minaker
    ## input_ex_smd.jl is free software; you can redistribute it and/or modify itππ


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

    the_system = mbd_system("Hanging chain")

    for i in 1:nbodys
        item = body("block_$i")
        item.mass = 1.0
        item.moments_of_inertia = [0.1, 0.1, 0.1]
        item.location = [0, 0, -nbodys + i - 1] * 0.5
        add_item!(item, the_system)
        add_item!(weight(item), the_system)
    end

    n = 2
    for i in 1:nbodys - 1
        for j in 1:n
            item = link("string_$i$j")
            item.body[1] = "block_$i"
            item.body[2] = "block_$(i + 1)"
            item.location[1] = [cos(2π * j / n), sin(2π * j / n), -nbodys + i - 1 + 0.25] * 0.5
            item.location[2] = [cos(2π * j / n), sin(2π * j / n), -nbodys + i - 1 + 0.75] * 0.5
            add_item!(item, the_system)
        end
    end

    for j in 1:n
        item = link("string_0$j")
        item.body[1] = "block_$nbodys"
        item.body[2] = "ground"
        item.location[1] = [cos(2π * j / n), sin(2π * j / n), -0.75] * 0.5
        item.location[2] = [cos(2π * j / n), sin(2π * j / n), -0.25] * 0.5
        add_item!(item, the_system)
    end


    # the actuator is a `line item` and defined by two locations, location[1] attaches to body[1]...
    item = actuator("X")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [-1, 0, -nbodys] * 0.5
    add_item!(item, the_system)

    # the sensor is also `line item` and defined by two locations, location[1] attaches to body[1]...
    item = sensor("kx")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [-1, 0, -nbodys] * 0.5
    item.gain = 9.81
    item.units = "N"
    add_item!(item, the_system)

    # the actuator is a `line item` and defined by two locations, location[1] attaches to body[1]...
    item = actuator("Y")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [0, -1, -nbodys] * 0.5
    add_item!(item, the_system)

    # the sensor is also `line item` and defined by two locations, location[1] attaches to body[1]...
    item = sensor("ky")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [0, -1, -nbodys] * 0.5
    item.gain = 9.81
    item.units = "N"
    add_item!(item, the_system)

    # the actuator is a `line item` and defined by two locations, location[1] attaches to body[1]...
    item = actuator("N")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [0, 0, -nbodys - 1] * 0.5
    item.twist = true
    item.units = "N*m"
    add_item!(item, the_system)

    # the sensor is also `line item` and defined by two locations, location[1] attaches to body[1]...
    item = sensor("kψ")
    item.body[1] = "block_1"
    item.body[2] = "ground"
    item.location[1] = [0, 0, -nbodys] * 0.5
    item.location[2] = [0, 0, -nbodys - 1] * 0.5
    item.twist = true
    item.gain = 9.81
    item.units = "N*m"
    add_item!(item, the_system)

    the_system

end
