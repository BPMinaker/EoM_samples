module quarter_car

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

# here we set the input as a random road where z is a function of distance x; it is a sum of 500 sin waves with random phase angles, and ampltiude decreasing as wavelength shortens; it will be different each time you run the code; the longest wavelength in the sum is the full length of the road, a default of 100 m; the wavelengths shorten as the sequence 100/2, 100/3, 100/4,..., with the shortest wavelength at 100/2000, or 20 cm; the class here is the road roughness, an integer ranging from 3-9.  A class 3 road is very smooth (on the boundary of ISO classes A and B), where class 9 is extremely rough (boundary of ISO classes G and H); the random road function returns a function handle that gives back `z` as a function of `x`
zofx = random_road(class=5)

# but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
u_vec(_, t) = [zofx(10 * t)]

println("Solving time history...")
t1 = 0
t2 = 10
yoft = ltisim(result.ss_eqns, u_vec, (t1, t2))
# default is to solve at 1000 time steps, giving a dt of 0.01 s, or a dx of 0.1 m, giving a minimum of two points per wavelength at the shortest wave (Nyquist criterion), and many more at longer wavelengths

# plot sprung mass
yidx = [1]
label, ylabel = ltilabels(system; yidx)
p1 = ltiplot(yoft; ylabel, label, yidx)

# plot suspension travel
yidx = [2]
label, ylabel = ltilabels(system; yidx)
p2 = ltiplot(yoft; ylabel, label, yidx)

# plot tire compression
yidx = [3]
label, ylabel = ltilabels(system; yidx)
p3 = ltiplot(yoft; ylabel, label, yidx)

plots = [p1, p2, p3]

impulse = :skip
#summarize(system, result; plots, impulse)
summarize(system, result; plots, impulse, format = :html)


# generate animations of the mode shapes
using EoM_X3D
animate_modes(system, result, scale=0.2)

end

println("Done.")
