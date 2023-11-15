using EoM, Plots, ForwardDiff
plotly()

include(joinpath("models", "input_ex_roll_centre.jl"))

# see the input file for the list of parameters and their default values
# build the list of parameters, and set those that are not using defaults
# here the value of m and u are set, but we can add many more if we like
m = 1500
u = 20
cfy = 0
cry = 0
params = params_list(; m, u, cfy, cry) # make sure to include them all here!!!

# build system description with approximate cornering stiffnesses
system = input_full_car_rc(; params)

# generate eom
output = run_eom!(system, true)

# get static tire normal loads (kN)
# assume LF, LR, RF, RR sequence
# display(getfield.(system.flex_points,:name))
Z0 = vcat(getfield.(system.flex_points[[1, 2, 5, 6]], :preload)...)

# define a nonlinear tire with load sensitivity
function tire(Z, slip)
    mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / pi, 6494.6, 4.7966E-3 * 180 / pi, -0.3875E-3, 1.0]
    C = mtm[1]
    D = (mtm[2] * Z .+ mtm[3]) .* Z
    B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
    Bslip = B .* slip
    E = mtm[7] * Z .+ mtm[8]

    -D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
end

# recompute cornering stiffnesses
# define the lateral force function at the actual vertical load
yf(x) = tire(Z0[1], x)
# differentiate by slip angle
dyf(x) = ForwardDiff.derivative(yf, x)
# evaluate a slip angle of zero
params.cfy = -dyf(0)
# repeat for rear
yr(x) = tire(Z0[2], x)
dyr(x) = ForwardDiff.derivative(yr, x)
params.cry = -dyr(0)

# rebuild the equations of motion using the updated cornering stiffnesses
system = input_full_car_rc(;params)
output = run_eom!(system, true)
result = analyze(output, true)

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
steer(t) = 3 * (0.5 * tanh(4 * (t - 1.5)) + 0.5)

# assume LF, LR, RF, RR sequence
# compute applied tire force
function input(x, t)
    # get sensor outputs (D=0)
    y = result.ss_eqns.C * x
    # get total normal load
    Z = Z0 - y[[1, 2, 5, 6]]
    # get slip angles from lateral velocity, subtract steer on front
    slip = y[[3, 4, 7, 8]] .- steer(t) * [1, 0, 1, 0] * π / 180
    # compute tire force, cancel linear tire
    tire(Z, slip) + [params.cfy, params.cry, params.cfy, params.cry] .* y[[3, 4, 7, 8]]
end

println("Solving time history...")
t = 0:0.002:8

y = splsim(result.ss_eqns, input, t)
y = hcat(y...)'

t = t[1:4:end]
y = y[1:4:end, :]

ZZ = Z0' .- y[:, [1, 2, 5, 6]]
slip = y[:, [3, 4, 7, 8]] - steer.(t) .* [1, 0, 1, 0]' * π / 180

YY = tire.(ZZ, slip)
acc = sum(YY, dims=2) / (params.m + 2 * params.muf + 2 * params.mur)
slip *= 180 / π

# set plot text, etc
lw = 2 # thicker plot lineweight
size = (800, 400)
xlims = (0, Inf)
xlabel = "Time [s]"
label = ["LF" "LR" "RF" "RR"]
plots = []

ylabel = "Tire slip α [°]"
push!(plots, plot(t, slip; xlabel, ylabel, label, lw, size, xlims))

ylabel = "Lateral forces ΣY [N]"
push!(plots, plot(t, YY; xlabel, ylabel, label, lw, size, xlims))

ylabel = "Vertical forces ΣZ [N]"
push!(plots, plot(t, ZZ; xlabel, ylabel, label, lw, size, xlims))

label = ["F" "R"]
ylabel = "Lateral weight transfer [N]"
push!(plots, plot(t, 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]; xlabel, ylabel, label, lw, size, xlims))

label = ""
ylabel = "Yaw moment N [Nm]"
push!(plots, plot(t, sum(params.a * YY[:, [1, 3]] - params.b * YY[:, [2, 4]], dims=2); xlabel, ylabel, label, lw, size, xlims))

ylabel = "G Lift [mm]"
push!(plots, plot(t, y[:, 10]; xlabel, ylabel, label, lw, size, xlims))

label = ["Steer δ" "Roll ϕ" "Pitch θ" "Slip β" "Understeer"]
ylabel = "Angles [°]"
push!(plots, plot(t, [steer.(t) y[:, 11:13] -y[:, 9] * (params.a + params.b) / params.u + steer.(t)]; xlabel, ylabel, label, lw, size, xlims))

label = ["ru" "Σf/m" "vdot"]
ylabel = "acc [m/s^2]"
push!(plots, plot(t, [y[:, 14] acc acc - y[:, 14]]; xlabel, ylabel, label, lw, size, xlims))

# pick the outputs for the Bode plots
bode = zeros(16, 4)
bode[15, :] = [1, 1, 1, 1]
ss = bode


summarize(system, result; plots, bode, ss)
# summarize(system, result; plots, format = :html)

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, true)

println("Done.")
