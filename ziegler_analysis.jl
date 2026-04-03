using EoM
using Plots
plotlyjs()

include(joinpath("models", "input_ex_ziegler.jl"))

function main()
    # Parameters
    m = 1.0
    l = 1.0
    k = 100.0
    c = 0.05 # Low damping to see instability clearly

    # Calculate critical flutter load (Theoretical approximation for Ziegler column is P_crit approx 2-3 * k/l depending on params)

    p_vals = 0.0:3.0:300.0

    systems = [input_ex_ziegler(; P=p, m, l, k, c) for p in p_vals]
    outputs = run_eom!.(systems, p_vals .== 0.0)
    results = analyze.(outputs, p_vals .== 0.0; impulse=:skip, bode=:skip)

    summarize(p_vals, results; vpt_name=["P" "Compressive load" "N"])

end

println("Starting Ziegler's Column analysis...")
main()
println("Done.")
