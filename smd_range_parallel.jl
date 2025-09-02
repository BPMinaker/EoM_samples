using EoM

# this file repeats the spring mass damper example, but shows how we can analyze a series of systems

include(joinpath("models", "input_ex_smd.jl"))

k = 1.0
m = 1.0

# here we redefine the input function, so we can call it using any value of c
f(x) = input_ex_smd(; k, m, c=x)

# then we define the range of values for c
vpts = 0:0.001:1

# timing experiment
# looping vs vectorizing vs piping vs chaining

id = zeros(length(vpts))

@time begin
    for i in eachindex(id)
        id[i] = Threads.threadid()
        system = f(vpts[i])
        output = run_eom!(system)
        result = analyze(output)
    end
end
#println(id)

@time begin
    Threads.@threads for i in eachindex(id)
        id[i] = Threads.threadid()
        system = f(vpts[i])
        output = run_eom!(system)
        result = analyze(output)
    end
end
#println(id)

# @time begin
#     system = f.(vpts)
#     output = run_eom!.(system)
#     result = analyze.(output)
# end

@time begin
    system = [input_ex_smd(; k, m, c=x) for x in vpts]
    output = run_eom!.(system)
    result = analyze.(output)
end

sdfdssdf()

@time begin
    result = vpts .|> f .|> run_eom! .|> analyze
end

@time begin
    result = (analyze ∘ run_eom! ∘ f).(vpts)
end

@time begin
    result = (analyze ∘ run_eom! ∘ (x -> input_ex_smd(; k, m, c=x))).(vpts)
end

# looping and vectorizing with anonymous function inline

@time begin
    for i in vpts
        system = (x -> input_ex_smd(; k, m, c=x))(i)
        output = run_eom!(system)
        result = analyze(output)
    end
end

@time begin
    system = (x -> input_ex_smd(; k, m, c=x)).(vpts)
    output = run_eom!.(system)
    result = analyze.(output)
end

@time begin
    system = map(x -> input_ex_smd(; k, m, c=x), vpts)
    output = run_eom!.(system)
    result = analyze.(output)
end

@time begin
    result = vpts .|> (x -> input_ex_smd(; k, m, c=x)) .|> run_eom! .|> analyze
end

@time begin
    g(x) = analyze(run_eom!(input_ex_smd(; k, m, c=x)))
    result = g.(vpts)
end



println("Done.")
