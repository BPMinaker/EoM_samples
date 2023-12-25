using EoM

# this file repeats the spring mass damper example, but shows how we can analyze a series of systems, using Julia's dot notation

include(joinpath("models", "input_ex_smd.jl"))

k = 1.0
m = 1.0

# here we redefine the input function, so we can call it using any value of c
f(x) = input_ex_smd(; k, m, c = x)

# then we define the range of values for c
vpts = 0:0.001:5

# timing experiment
# looping vs vectorizing

@time begin
for i in vpts
    system = f(i)
    output = run_eom!(system)
    result = analyze(output)
end
end

@time begin
    system = f.(vpts)
    output = run_eom!.(system)
    result = analyze.(output)
end


println("Done.")