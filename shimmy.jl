using EoM
include(joinpath("models", "input_ex_shimmy.jl"))

function main()

    m = 5
    k = 0.3 * m
    l = 1
    u = l
    I = 0.21 * m * l^2
    vpts = range(0, 1; length=151)
    vpt_name = ["a/l" "Mass centre location" ""]

    system = [input_ex_shimmy(; m, k, a=x * l, b=l - (x * l), u, I) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output)

    ss = :skip
    impulse = :skip
    summarize(system, vpts, result; impulse, ss, vpt_name)

    println("Done.")

end

main()


# error check
# r2 = 0.21 * l^2
# μ(x) = (x * l * (l -  x * l) - r2)/((x * l)^2 + r2)
# ωn(x) = sqrt((k * l^2)/ m /((x * l)^2 + r2))

# for i = 1:length(vpts)
#     λ = result[i].e_val[2]
#     display(λ^3 + (1 + μ(vpts[i])) * (u/l) * λ^2 + ωn(vpts[i])^2 * λ + (u/l) * ωn(vpts[i])^2)
# end
