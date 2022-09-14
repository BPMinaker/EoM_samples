using EoM

build_examples()
include(joinpath("examples", "input_ex_shimmy.jl"))

m = 5
k = 0.3 * m
l = 1
u = l
I = 0.21 * m * l^2

f(x) = input_ex_shimmy(;m, k, a = x * l, b = l - (x * l), u, I)
vpts = 0:0.01:1

system = f.(vpts)
output = run_eom!.(system)
result = analyze.(output)

summarize(system, vpts, result, vpt_name = ["a/l" "Mass centre location" ""])
# summarize(system, vpts, result, vpt_name = ["a/l" "Mass centre location" ""], format = :html)

println("Done.")

# error check
# r2 = 0.21 * l^2 
# μ(x) = (x * l * (l -  x * l) - r2)/((x * l)^2 + r2)
# ωn(x) = sqrt((k * l^2)/ m /((x * l)^2 + r2))

# for i = 1:length(vpts)
#     λ = result[i].e_val[2]
#     display(λ^3 + (1 + μ(vpts[i])) * (u/l) * λ^2 + ωn(vpts[i])^2 * λ + (u/l) * ωn(vpts[i])^2)
# end