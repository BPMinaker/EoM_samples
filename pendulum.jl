using EoM

build_examples()
include(joinpath("examples", "input_ex_pendulum.jl"))

system = input_ex_pendulum()

output = run_eom!(system)
result = analyze(output)

summarize(system, result)
write_html(system, result)
write_output(system, result)


# using EoM_X3D
# animate_modes(system, result())

println("Done.")
