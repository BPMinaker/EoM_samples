using EoM

# this file repeats the spring mass damper example, but shows how we can analyze a series of systems, using Julia's dot notation

include(joinpath("models", "input_ex_smd.jl"))

k = 1.0
m = 1.0

# here we redefine the input function, so we can call it using any value of c
f(x) = input_ex_smd(; k, m, c = x)

# then we define the range of values for c
vpts = 0:0.001:5

@time begin
    n = length(vpts)
    system = Vector{EoM.mbd_system}(undef, n)
    output = Vector{EoM.dss_data}(undef, n)
    result = Vector{EoM.analysis}(undef, n)

    Threads.@threads for i in 1:n
        system[i] = f(vpts[i])
        output[i] = run_eom!(system[i])
        result[i] = analyze(output[i])
    end
end

@time begin
    system2 = f.(vpts)
    output2 = run_eom!.(system2)
    result2 = analyze.(output2)
end


@time begin
    result3 = analyze.(run_eom!.(f.(vpts)))
end




# we could also write to html output instead of the screen
# summarize(system, vpts, result; ss = [], bode = [0,0,1], vpt_name = ["c" "Damping coefficient" "Ns/m"], format = :html)

println("Done.")