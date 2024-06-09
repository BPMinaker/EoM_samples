using EoM, ForwardDiff

include(joinpath("models", "input_ex_roll_centre.jl"))

# see the input file for the list of parameters and their default values
# set the list of parameters that are not using defaults
# here the value of m and u are set, but we can add many more if we like
m = 1500
u = 80.5 / 3.6
cfy = 0
cry = 0
hf = 0.3
hr = 0.3

# build system description with approximate cornering stiffnesses
system = input_full_car_rc(; m, u, cfy, cry, hf, hr) # make sure to include all parameters here, and again below!!!

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
cfy = -dyf(0)
# repeat for rear
yr(x) = tire(Z0[2], x)
dyr(x) = ForwardDiff.derivative(yr, x)
cry = -dyr(0)

# rebuild the equations of motion using the updated cornering stiffnesses
system = input_full_car_rc(; m, u, cfy, cry, hf, hr)
output = run_eom!(system, true)
result = analyze(output, true)

# a smooth step function, 3 degrees, with its midpoint (i.e. 1.5) at t = 2
# the 0.5 terms change the range from -1...1 to 0...1
# the 4 determines how quickly the step occurs
# steer(t) = 3 * (0.5 * tanh(4 * (t - 1.5)) + 0.5)

# a sin w dwell input ala FMVSS 126
steer(t) = 2 * (EoM.pulse(t, 2, 2 + 1/0.7*0.75) * sin(2π * 0.7 * (t - 2)) - EoM.pulse(t, 2 + 1/0.7*0.75, 2.5 + 1/0.7*0.75) + EoM.pulse(t, 2.5 + 1/0.7*0.75, 2.5 + 1/0.7) * sin(2π * 0.7 * (t - 2.5)))

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
    tire(Z, slip) + [cfy, cry, cfy, cry] .* y[[3, 4, 7, 8]]
end

println("Solving time history...")
t = 0:0.002:8

y = splsim(result.ss_eqns, input, t)

t = t[1:4:end]
y = Matrix(y[1:4:end])

ZZ = Z0' .- y[:, [1, 2, 5, 6]]
slip = y[:, [3, 4, 7, 8]] - steer.(t) .* [1, 0, 1, 0]' * π / 180

YY = tire.(ZZ, slip)
acc = sum(YY, dims = 2) * 9.81 / sum(Z0) 
slip *= 180 / π


delta = steer.(t)
# set plot text, etc
lw = 2 # thicker plot lineweight
size = (800, 400)
xlabel = "Time [s]"
label = ["LF" "LR" "RF" "RR"]
plots = []

ylabel = "Tire slip α [°]"
push!(plots, plot(t, slip; xlabel, ylabel, label, lw, size))

ylabel = "Lateral forces ΣY [N]"
push!(plots, plot(t, YY; xlabel, ylabel, label, lw, size))

ylabel = "Vertical forces ΣZ [N]"
push!(plots, plot(t, ZZ; xlabel, ylabel, label, lw, size))

label = ["F" "R"]
ylabel = "Lateral weight transfer [N]"
push!(plots, plot(t, 0.5 * [ZZ[:, 3] - ZZ[:, 1] ZZ[:, 4] - ZZ[:, 2]]; xlabel, ylabel, label, lw, size))

label = ""
ylabel = "Yaw moment N [Nm]"
push!(plots, plot(t, sum(system.scratch.a * YY[:, [1, 3]] - system.scratch.b * YY[:, [2, 4]], dims = 2); xlabel, ylabel, label, lw, size))

ylabel = "G Lift [mm]"
push!(plots, plot(t, y[:, 10]; xlabel, ylabel, label, lw, size))

label = ["Steer δ" "Roll ϕ" "Slip β" "Understeer α_u"]
ylabel = "Angles [°]"
push!(plots, plot(t, [delta y[:, [11 ,13]] y[:, 14] .+ delta]; xlabel, ylabel, label, lw, size))

label = ["Steer δ" "Pitch θ"]
ylabel = "Angles [°]"
push!(plots, plot(t, [delta y[:, 12]]; xlabel, ylabel, label, lw, size))

label = ["ru" "Σf/m" "vdot"]
ylabel = "acc [m/s^2]"
push!(plots, plot(t, [y[:, 15] acc acc - y[:, 15]]; xlabel, ylabel, label, lw, size))

label = ["Steer δ" "Yaw rate r"]
ylabel = "Angular speed [°/s]"
push!(plots, plot(t, [delta y[:, 9]]; xlabel, ylabel, label, lw, size))

bode = 0 * result.ss_eqns.D
impulse = bode
ss = bode

summarize(system, result; plots, bode, ss, impulse, format = :html)

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, true)

println("Done.")
