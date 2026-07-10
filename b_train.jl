using EoM, EoM_X3D
using Plots
plotlyjs()

format = :screen
# format = :html

include(joinpath("models", "input_ex_b_train.jl"))

function main()

    # B-train parameters
    u = 20
    # Tractor
    m1 = 7000
    Iz1 = 3500
    # Lead Trailer
    m2 = 8000
    Iz2 = 4000
    # Pup Trailer
    m3 = 8000
    Iz3 = 4000

    # Speed range for analysis
    vpts = 0.4:0.4:40

    # Create system instances for different speeds
    system = [input_ex_b_train(; u=x, m1, Iz1, m2, Iz2, m3, Iz3) for x in vpts]

    # Run EoM generation
    output = run_eom!.(system, vpts .== 1)

    # Analyze stability (linear analysis)
    result = analyze.(output, vpts .== 1; freq=(-1, 1), impulse=:skip, bode=:skip)

    # Summarize results
    summarize(vpts, result; format)


    # Time history simulation at specific speed (e.g., 10 m/s)
    # Choose the equations of motion for 10 m/s
    u_sim = 6
    # Find closest speed in vpts
    n = findfirst(abs.(vpts .- u_sim) .< 0.1)
    if isnothing(n)
        error("Simulation speed $u_sim not found in vpts range")
    end

    animate_modes(system[n], result[n])

    sys_sim = system[n]
    res_sim = result[n]
    sys_sim.name *= " $(vpts[n]) m/s"

    # Define input: Steer input (sine pulse)
    # steer(t) = EoM.pulse(t, 1, 3) * 2 * sin(π * (t - 1))

    # Simple lane change-like steer input
    function steer_input(t)
        if 1.0 <= t <= 3.0
            return 2.0 * sin(π * (t - 1.0))
        else
            return 0.0
        end
    end

    # Input vector function
    u_vec(_, t) = [steer_input(t)]

    # Define time interval
    t1 = 0
    t2 = 10

    # Solve the equations of motion
    yoft = ltisim(res_sim, u_vec, (t1, t2))

    plots = [ltiplot(yoft; sidx=i) for i in[["r1", "r2", "r3"], ["β", "γ1", "γ2"], ["ay1"]]]

    # Show summary with plots
    summarize(res_sim; plots, format)

end

println("Starting B-train analysis...")
main()
println("Done.")
