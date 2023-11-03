using EoM, Plots
plotly()

include(joinpath("models", "input_quarter_car_planar_a_arm.jl"))
include(joinpath("models", "tire.jl"))
include("kinematics.jl")


# here you can enter your vehicle specs by name
a = 2.65 * 0.58
tw = 1.7
r = 0.32
u = 10
vpt = 2500:-50:-5000
ss = []
vpt_name = ["Y" "Lateral Force" "N"]
t = 0:0.002:2

ptr(x) = input_quarter_car_planar_a_arm(; u, a, tw, r, Y = x)
system = ptr.(vpt)
setfield!.(system, :name, "Quarter Car A-arm - long arms")
output = run_eom!.(system)
result = analyze.(output)

rr = system[1].scratch[1]
L = system[1].scratch[2]
rc = system[1].scratch[3]
display(rr)


h1 = impulse(result[end].ss_eqns, t)
h1 = 1e-4 * hcat(h1...)
h4 = impulse(result[1].ss_eqns, t)
h4 = 1e-4 * hcat(h4...)


summarize(system, vpt, result; vpt_name, ss)


##########

ptr(x) = input_quarter_car_planar_a_arm(; u, a, tw, r, Y = x, wui = 0.53, wli = 0.53)
system = ptr.(vpt)
output = run_eom!.(system)
result = analyze.(output)

rr = system[1].scratch[1]
L = system[1].scratch[2]
rc = system[1].scratch[3]
display(rr)


h2 = impulse(result[end].ss_eqns, t)
h2 = 1e-4 * hcat(h2...)
h5 = impulse(result[1].ss_eqns, t)
h5 = 1e-4 * hcat(h5...)


summarize(system, vpt, result; vpt_name, ss)


##########

ptr(x) = input_quarter_car_planar_a_arm(; u, a, tw, r, Y = x, wui = 0.53)
system = ptr.(vpt)
setfield!.(system, :name, "Quarter Car A-arm - mixed arms")
output = run_eom!.(system)
result = analyze.(output)

rr = system[1].scratch[1]
 L = system[1].scratch[2]
rc = system[1].scratch[3]
display(rr)

h3 = impulse(result[end].ss_eqns, t)
h3 = 1e-4 * hcat(h3...)
h6 = impulse(result[1].ss_eqns, t)
h6 = 1e-4 * hcat(h6...)



size = (800, 600)
lw = 2
p1 = plot(t, [h1[1,:], h2[1,:], h3[1,:]]; size, lw)
p2 = plot(t, [h4[1,:], h5[1,:], h6[1,:]]; size, lw)


plots = [p1, p2]

summarize(system, vpt, result; plots, vpt_name, ss)



# summarize(system[end], result[end]; ss)
# summarize(system, vpt, result; vpt_name, ss)
# animate_modes(system[end], result[end], overwrite = false)


# summarize(system, vpt, result; vpt_name, ss)
# animate_modes(system[1], result[1], overwrite = false)


println("Done.")

