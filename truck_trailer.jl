using EoM
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_truck_trailer.jl"))

function main()

    # here you can enter your vehicle specs by name
    # you can add a, b, cf, cr, etc, as long as they are added to the arguments of input_ex_truck_trailer function
    m = 16975 / 9.81 # truck mass
    Iz = 3508 # truck inertia
    d = 2.7 # hitch distance
    e = 2.9 # trailer front wheelbase
    h = 0.1 # trailer rear wheelbase
    mt = 2000 # trailer mass
    It = 3000 # trailer inertia
    vpts = 0.4:0.4:40

    system = [input_ex_truck_trailer(; u=x, m, Iz, d, e, h, mt, It) for x in vpts]
    output = run_eom!.(system, vpts .== 1)

    result = analyze.(output, vpts .== 1; freq=(-1, 1), impulse=:skip)
    summarize(vpts, result; format)

    # choose the equations of motion for 18 m/s (note function notation)
    n = findfirst(vpts .== 18)
    system = system[n]
    result = result[n]
    system.name *= " 18 m per s"

    # equations are known, let's solve a time history
    steer(t) = EoM.pulse(t, 1, 3) * 2 * sin(π * (t - 1))
    u_vec(_, t) = [steer(t)] # define input function to be steer but to also accept x and then ignore it

    # Define time interval
    t1 = 0
    t2 = 20

    # solve the equations of motion
    yoft = ltisim(result, u_vec, (t1, t2))

    # sensors are, in order, r, β, α_u, ψ, a_lat

    # plot yaw rate vs time
    sidx = ["r"]
    plots = [ltiplot(yoft; sidx)]

    # plot body slip angle vs time
    sidx = ["β"]
    push!(plots, ltiplot(yoft; sidx))

    # plot understeer angle vs time
    sidx = ["α_u"]
    push!(plots, ltiplot(yoft; sidx))

    # plot trailer sway angle vs time
    sidx = ["γ"]
    push!(plots, ltiplot(yoft; sidx))

    # plot lateral acceleration vs time
    sidx = ["a_y"]
    push!(plots, ltiplot(yoft; sidx))

    summarize(result; plots, format)

    # animate_modes(system, result)

end

println("Starting...")
main()
println("Done.")
