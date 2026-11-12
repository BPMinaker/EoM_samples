using EoM
using Plots
plotlyjs()

format = :screen
#format = :html

include(joinpath("models", "input_ex_bounce_pitch.jl"))
include(joinpath("models", "input_ex_quarter_car.jl"))
include(joinpath("models", "input_ex_half_car.jl"))

    m = 2000
    a = 1.5
    b = 1.3
    kf = 25000
    kr = 30000
    cf = 1500
    cr = 1600
    Iy = 2500

    muf = 100
    mur = 150
    kt = 300000

    u = 20 # vehicle speed, used to compute delay from front to rear axle

    function main()

    system = input_ex_bounce_pitch(;m, a, b, kf, kr, cf, cr, Iy)
    # note that we don't pass u here, it is only used later for delay from front to rear axle

    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse, freq=(-1,2))

    input_delay!(result, (a + b) / u, [1, 2]) # this function modifies the frequency response to include the input delay (multiplies second input by exp(-iϕ))

    nin = length(result.sys_data.anames)
    nout = length(result.sys_data.snames)
    bode = zeros(nout,nin)
    bode[result.sys_data.sidx["z_f"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_r"], result.sys_data.aidx["u_f"]] = 1

    summarize(result; bode=:skip, impulse, ss=:skip, format)

    # mag is a vector of matrices, we want to build vectors from certain locations inside each matrix    
    temp_i = [temp[bode .== 1] for temp in result.mag]
    # temp_i is a vector of vectors, we want a matrix
    mag = hcat(temp_i...)'

    w = result.w / 2π
    xticks = w[isinteger.(log10.(w))]

    # now do quarter car

    ms_f = m * b/(a+b)
    ms_r = m * a/(a+b)
    system_f = input_ex_quarter_car(;ms=ms_f, mu=muf, ks=kf, cs=cf, kt) 
    system_r = input_ex_quarter_car(;ms=ms_r, mu=mur, ks=kr, cs=cr, kt) 

    output_q = run_eom!.([system_f, system_r])
    result_q = analyze.(output_q; impulse)

    nin = length(result_q[1].sys_data.anames)
    nout = length(result_q[1].sys_data.snames)
    bode_q = zeros(nout,nin)
    bode_q[result_q[1].sys_data.sidx["z_s"], result_q[1].sys_data.aidx["z_g"]] = 1 

    summarize(result_q[1]; bode=:skip, impulse, ss=:skip, format)
    summarize(result_q[2]; bode=:skip, impulse, ss=:skip, format)

    # we know there is only one entry from the matrix we want, but we get a vector of length one, so we need to pick it
    mag_f = [temp[bode_q .== 1][1] for temp in result_q[1].mag]
    mag_r = [temp[bode_q .== 1][1] for temp in result_q[2].mag]


    system = input_ex_half_car(;m, a, b, kf, kr, cf, cr, Iy, muf, mur, kt)
    # note that we don't pass u here, it is only used later for delay from front to rear axle

    output = run_eom!(system)
    impulse = :skip
    result = analyze(output; impulse, freq=(-1,2))

    input_delay!(result, (a + b) / u, [1, 2]) # this function modifies the frequency response to include the input delay (multiplies second input by exp(-iϕ))

    nin = length(result.sys_data.anames)
    nout = length(result.sys_data.snames)
    bode = zeros(nout,nin)
    bode[result.sys_data.sidx["z_f"], result.sys_data.aidx["u_f"]] = 1 
    bode[result.sys_data.sidx["z_r"], result.sys_data.aidx["u_f"]] = 1

    summarize(result; bode=:skip, impulse, ss=:skip, format)

    # mag is a vector of matrices, we want to build vectors from certain locations inside each matrix    
    temp_i = [temp[bode .== 1] for temp in result.mag]
    # temp_i is a vector of vectors, we want a matrix
    mag_h = hcat(temp_i...)'

    display(plot(w, [mag[:,1] mag_f mag_h[:,1]] ; xscale = :log10, xticks, ylims=(-40,Inf), xlabel="Freq [Hz]", ylabel="Gain [dB]",labels=["BP z_f/z_g" "QC z_f/z_g" "HC z_f/z_g"], widen=true))

    display(plot(w, [mag[:,2] mag_r mag_h[:,2]] ; xscale = :log10, xticks, ylims=(-40,Inf), xlabel="Freq [Hz]", ylabel="Gain [dB]",labels=["BP z_r/z_g" "QC z_r/z_g"  "QC z_r/z_g"], widen=true))

end

println("Starting...")
main()
println("Done.")
