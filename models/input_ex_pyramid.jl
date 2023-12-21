function input_ex_pyramid(; rows = 5)

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


    the_system = mbd_system("Pyramid")

    for i in 1:rows
        for j in 1:i
            item = body("block_$i$j")
            item.mass = 1.0
            item.moments_of_inertia = [0.1, 0.1, 0.1]
            temp = [0.3, 0, -0.3] * i + [0.3, 0, 0.3] * j - [1.8, 0, 0.3]
            item.location = temp
            add_item!(item, the_system)
            add_item!(weight(item), the_system)

            item = link("string_$i$j 1")
            item.body[1] = "block_$i$j"
            item.body[2] = "block_$(i-1)$j"
            item.location[1] = temp + [-0.1, 0.1, 0.1]
            item.location[2] = temp + [-0.2, 0.1, 0.2]
            i == j && (item.body[2] = "ground")
            add_item!(item, the_system)

            item = link("string_$i$j 2")
            item.body[1] = "block_$i$j"
            item.body[2] = "block_$(i-1)$j"
            item.location[1] = temp + [-0.1, -0.1, 0.1]
            item.location[2] = temp + [-0.2, -0.1, 0.2]
            i == j && (item.body[2] = "ground")
            add_item!(item, the_system)

            item = link("string_$i$j 3")
            item.body[1] = "block_$i$j"
            item.body[2] = "block_$i$(j+1)"
            item.location[1] = temp + [0.1, 0.1, 0.1]
            item.location[2] = temp + [0.2, 0.1, 0.2]
            i == j && (item.body[2] = "ground")
            add_item!(item, the_system)

            item = link("string_$i$j 4")
            item.body[1] = "block_$i$j"
            item.body[2] = "block_$i$(j+1)"
            item.location[1] = temp + [0.1, -0.1, 0.1]
            item.location[2] = temp + [0.2, -0.1, 0.2]
            i == j && (item.body[2] = "ground")
            add_item!(item, the_system)

        end
    end

    item = actuator("Y")
    item.body[1] = "block_11"
    item.body[2] = "ground"
    item.location[1] = [-1.2, 0, -0.3]
    item.location[2] = [-1.2, 0.1, -0.3]
    add_item!(item, the_system)

    item = sensor("ky")
    item.body[1] = "block_11"
    item.body[2] = "ground"
    item.location[1] = [-1.2, 0, -0.3]
    item.location[2] = [-1.2, 0.1, -0.3]
    item.gain = 9.81
    item.units = "N"
    add_item!(item, the_system)

    the_system

end



