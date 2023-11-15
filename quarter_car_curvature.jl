using EoM, Plots
plotly()
size = (800, 400)
lw = 2
xlabel = "Time [s]"
ylabel = "Motion [m]"

include(joinpath("models", "input_quarter_car_planar_a_arm.jl"))
include(joinpath("models", "tire.jl"))
include("kinematics.jl")

vpt = 2500:-50:-5000
idx = [1, 51, 101, 151]
label = reshape(["Y=$x N" for x in vpt[idx]], 1, :)

ss = []
vpt_name = ["Y" "Lateral Force" "N"]
t = 0:0.005:2

# here you can enter your vehicle specs by name
a = 2.65 * 0.58
tw = 1.7
wuo = tw / 2 - 0.05
wlo = tw / 2 - 0.05
r = 0.32
u = 10

ptr(x) = input_quarter_car_planar_a_arm(; params=params_list(; a, tw, r, u, wuo, wlo, Y=x))
system = ptr.(vpt)
setfield!.(system, :name, "Quarter Car A-arm - long arms")
output = run_eom!.(system)
result = analyze.(output)

rr, L, rc = system[1].scratch
display(rr)

h=[]
hh = impulse.(getfield.(result, :ss_eqns), [t])
for i in hh[idx]
    push!(h, hcat(i...)[1,:])
end

plots = [plot(t, hcat(h...); xlabel, ylabel, label, size, lw)]
summarize(system, vpt, result; plots, vpt_name, ss)

##########

ptr(x) = input_quarter_car_planar_a_arm(; params=params_list(; a, tw, r, u, Y=x, wuo, wlo, wui=0.53, wli=0.53))
system = ptr.(vpt)
output = run_eom!.(system)
result = analyze.(output)

rr, L, rc = system[1].scratch
display(rr)

h=[]
hh = impulse.(getfield.(result, :ss_eqns), [t])
for i in hh[idx]
    push!(h, hcat(i...)[1,:])
end

plots = [plot(t, hcat(h...); xlabel, ylabel, label, size, lw)]
summarize(system, vpt, result; plots, vpt_name, ss)

##########

ptr(x) = input_quarter_car_planar_a_arm(; params=params_list(; a, tw, r, u, Y=x, wuo, wlo, wui=0.53))
system = ptr.(vpt)
setfield!.(system, :name, "Quarter Car A-arm - mixed arms")
output = run_eom!.(system)
result = analyze.(output)

rr, L, rc = system[1].scratch
display(rr)

h=[]
hh = impulse.(getfield.(result, :ss_eqns), [t])
for i in hh[idx]
    push!(h, hcat(i...)[1,:])
end

plots = [plot(t, hcat(h...); xlabel, ylabel, label, size, lw)]
summarize(system, vpt, result; plots, vpt_name, ss)


# animate_modes(system[1], result[1], overwrite = false)

println("Done.")

