using EoM

include(joinpath("models", "input_ex_quarter_car.jl"))

# here you can enter your vehicle specs by name
ms = 500
mu = 50
kt = 150000
ks = 18000
cs = 500

verbose = true
system = input_ex_quarter_car(; ms, mu, kt, ks, cs)
output = run_eom!(system, verbose)
result = analyze(output, verbose)

# here we set the input as a random road where z is a function of distance x; it is a sum of 2000 sin waves with random phase angles, and ampltiude decreasing as wavelength shortens; it will be different each time you run the code; the longest wavelength in the sum is the full length of the road, a default of 100 m; the wavelengths shorten as the sequence 100/2, 100/3, 100/4,..., with the shortest wavelength at 100/2000, or 5 cm; the class here is the road roughness, an integer ranging from 3-9.  A class 3 road is very smooth (on the boundary of ISO classes A and B), where class 9 is extremely rough (boundary of ISO classes G and H); the random road function returns a function handle that gives back `z` as a function of `x`
zofx = random_road(class = 5)

# but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
u_vec(~, t) = zofx(10 * t)

# now define the time interval, and step size; at a forward speed of 10 m/s, this means the shortest wavelength is covered in 0.05/10 = 0.005 seconds per cycle, or occurs at 1/0.005 = 200 Hz (cycles / second); in order to capture this accurately, we need at least two time steps per wavelength, or a time step of 0.0025 s; ideally, we would have 10-20 steps per cycle, but in this case, that would be overdoing it, as 0.0025 s is already very short compared to the timescale of the vehicle response, which we would normally model up about to 50 Hz; a step of 0.0025 gives 8 samples in a 50 Hz cycle; this gives us lots of time steps per time constant and/or wavelength of the vehicle model; using that small time step means that we get 4001 solution points in a 10 s interval; a typical HD screen can only display 1920 pixels across, so there is no way we can display that much data in a graph without zooming in

println("Solving time history...")
t = 0:0.0025:10

# because our model is linear, we can use the built-in linear ODE solver in EoM (splsim); it's much simpler and faster than using the ODE toolbox
y = splsim(result.ss_eqns, u_vec, t)

z1 = y[:, 1]
z12 = y[:, 2]
z20 = y[:, 3]
z0 = u_vec.(0, t)

xlabel = "Time [s]"
ylabel = "Displacement [m]"
lw = 2
size = (800, 400)

label = ["Sprung mass" "Ground"]
plots = [plot(t, [z1 z0]; xlabel, ylabel, label, lw, size)]

label = ["Suspension travel" "Ground"]
push!(plots, plot(t, [z12 z0]; xlabel, ylabel, label, lw, size))

label = ["Tire compression" "Ground"]
push!(plots, plot(t, [z20 z0]; xlabel, ylabel, label, lw, size))

impulse = :skip
summarize(system, result; plots, impulse)
# summarize(system, result; plots, impulse, format = :html)
# uncomment to save to html

# generate animations of the mode shapes
# using EoM_X3D
# animate_modes(system, result, scale=0.2)

println("Done.")
