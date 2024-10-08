using EoM

include(joinpath("models", "input_n_dof.jl"))


f(x) = input_n_dof(;n=10, k=x)

vpts = 1:0.001:2

@time system = f.(vpts)
@time output = run_eom!.(system)
@time result = analyze.(output)

println("Done.")
