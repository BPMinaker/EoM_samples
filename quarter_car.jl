using EoM
# using EoM_X3D
include(joinpath("models", "input_ex_quarter_car.jl"))

function main()

    # here you can enter your vehicle specs by name
    ms = 500
    mu = 50
    kt = 150000
    ks = 18000
    cs = 500

    format = :screen
    # format = :html

    system = input_ex_quarter_car(; ms, mu, kt, ks, cs)
    output = run_eom!(system)
    result = analyze(output)

    # here we set the input as a random road where z is a function of distance x; it is a sum of 500 sin waves with random phase angles, and ampltiude decreasing as wavelength shortens; it will be different each time you run the code; the longest wavelength in the sum is the full length of the road, a default of 100 m; the wavelengths shorten as the sequence 100/2, 100/3, 100/4,..., with the shortest wavelength at 100/500, or 20 cm; the class here is the road roughness, an integer ranging from 3-9.  A class 3 road is very smooth (on the boundary of ISO classes A and B), where class 9 is extremely rough (boundary of ISO classes G and H); the random road function returns a function handle that gives back `z` as a function of `x`
    zofx = random_road(class=5)

    # but we need to convert to time index, where x=ut; assuming a forward speed of u=10 m/s gives
    u_vec(_, t) = [zofx(10 * t)]

    println("Solving time history...")
    t1 = 0
    t2 = 10
    yoft = ltisim(result, u_vec, (t1, t2))
    # default for ltisim is to solve at 1000 Hz, so at 10 m/s, a dx of 0.01 m, or 1 cm, giving 20 points per wavelength at the shortest wave, much more than the minimum of 2 points per wavelength (Nyquist criterion), and many more at longer wavelengths

    # plot sprung mass
    yidx = [1]
    p1 = ltiplot(system, yoft; yidx)
    # at 1000 Hz, a time interval of 10 s gives us 10000 points, which is fine until we want to plot on a screen with only 1920 pixels, so by default we downsample to plot a maximum 2000 points, unless you set an intger variable scale = x in the `ltiplot` call, where x is the number of points to skip; for example, `ltiplot(system, yoft; yidx, scale=50)` will plot every 50th point, or 200 points in total if the time interval is 10 s at 1000 Hz

    # plot suspension travel
    yidx = [2]
    p2 = ltiplot(system, yoft; yidx)

    # plot tire compression
    yidx = [3]
    p3 = ltiplot(system, yoft; yidx)

    plots = [p1, p2, p3]

    impulse = :skip
    summarize(system, result; plots, impulse, format)

    # generate animations of the mode shapes
    # animate_modes(system, result, scale=0.2)

end

println("Done.")

main()
