
using EoM, Plots
plotly()

build_examples()
include(joinpath("examples", "input_ex_quarter_car.jl"))

# Here you can enter your vehicle specs by name
temp() = input_ex_quarter_car(ms = 500, mu = 50, kt = 150000, ks = 18000, cs = 500)
my_sys, my_eqns = run_eom(temp, :verbose)
my_result = analyze(my_eqns, :verbose)

# Here we set the input as a random road where z is a function of distance x. It is a sum of 2000 sin waves with random phase angles, and ampltiude decreasing as wavelength shortens.  It will be different each time you run the code.  The longest wavelength in the sum is the full length of the road, a default of 100 m.  The wavelengths shorten as the sequence 100/2, 100/3, 100/4,..., with the shortest wavelength at 100/2000, or 5 cm.
zofx = random_road(class = 5)
# The class here is the road roughness, an integer ranging from 3-9.  A class 3 road is very smooth (on the boundary of ISO classes A and B), where class 9 is extremely rough (boundary of ISO classes G and H).  The random road function returns a function handle that gives back `z` as a function of `x`.  But we need to convert to time index, where x=ut.  Assuming a forward speed of u=10 m/s gives:
zoft(t) = zofx(10 * t)

# Now define the time interval, and step size.  At a forward speed of 10 m/s, this means the shortest wavelength is covered in 0.05/10 = 0.005 seconds.  In order to capture this accurately,we should have at least two time steps per wavelength, or a time step of 0.0025 s.  This turns out to be the limiting factor here, as 0.0025 s is very short compared to the timescale of the vehicle response (gives us lots of time steps per time constant and/or wavelength of the vehicle model).
t = 0:0.0025:10

# Compute the resulting road and store it
z = zoft.(t)

# Because our model is linear, we can use the built-in linear ODE solver in EoM (splsim).  It's much simpler and faster than using the ODE toolbox.
y = splsim(my_result[1].ss_eqns, z, t)
res = hcat(y...)

# Using that small time step means that we get about 4000 solution points. A typical HD screen can only display 1920 pixels across, so there is no way we can display that much data in a graph.  So, we filter out the data and plot, e.g., only every 4th point.  We use [1:4:end] to pick every 4th point.

t = t[1:4:end]
z0 = z[1:4:end]
z1 = res[1, 1:4:end]
z12 = res[2, 1:4:end]
z20 = res[3, 1:4:end]

xlabel = "Time [s]"
ylabel = "Displacement [m]"
lw = 2

label = ["Sprung mass" "Ground"]
p1 = plot(t, [z1 z0]; xlabel, ylabel, label, lw)

label = ["Suspension travel" "Ground"]
p2 = plot(t, [z12 z0]; xlabel, ylabel, label, lw)

label = ["Tire compression" "Ground"]
p3 = plot(t, [z20 z0]; xlabel, ylabel, label, lw)

write_html(my_sys, my_result, p1, p2, p3, :verbose)
println("Done.")
