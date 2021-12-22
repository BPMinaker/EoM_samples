using EoM

# this file repeats the spring mass damper example, but shows how we can analyze a series of systems, using Julia's dot notation

build_examples()
include(joinpath("examples", "input_ex_smd.jl"))

k = 1.0
m = 1.0

# here we redefine the input function, so we can call it using any value of c
f(x) = input_ex_smd(; k, m, c = x)

# then we define the range of values for c
vpts = 0:0.03:3

# then we call the function using the dot notation
system = f.(vpts)

# we take the vector of input systems, and generate the equations for all of them, again using dot notation
output = run_eom!.(system)

# the `analyze()` function has been written using another feature of Julia, called `multiple dispatch`, which allows the same function to do different things, depending on the type of arguments, analyze will behave differently if `output` is a vector of results, so we don't need the .()
result = analyze(output)


# summarize also recognizes that the result is a vector, so it drops the tables, and gives a different series of plots
summarize(system, vpts, result; ss = [], bode = [3], vpt_name = ["c" "Damping coefficient" "Ns/m"])

println("Done.")