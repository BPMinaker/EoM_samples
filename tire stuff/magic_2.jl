# magic formula tire model parameters

function tire(ZN, αr, γr)

    a0 = 1.799
    a1 = 0
    a2 = 1688
    a3 = 4140
    a4 = 6.026
    a5 = 0
    a6 = -0.3589
    a7 = 1
    a8 = 0
    a9 = -6.111E-3
    a10 = -3.224E-2
    a111 = 0
    a112 = 0
    a12 = 0
    a13 = 0

    Z = ZN / 1000  # convert to kN
    α = αr * 180 / π  # convert to degrees
    γ = γr * 180 / π  # convert to degrees

    a11 = a111 * Z .+ a112
    Sh = a8 * γ + a9 * Z .+ a10
    Sv = a11 .* γ .* Z + a12 * Z .+ a13

    C = a0
    D = (a1 * Z .+ a2) .* Z
    B = a3 * sin.(2 * atan.(Z / a4)) .* (1.0 .- a5 * abs.(γ)) / C ./ D
    Bslip = B .* (α + Sh)
    E = a6 * Z .+ a7

    Y = D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip))) + Sv

    -Y
end

function tire2(ZN, αr, γr)

    c0 = 2.068
    c1 = -6.49
    c2 = -21.85
    c3 = 0.416
    c4 = -21.31
    c5 = 2.942E-2
    c6 = 0
    c7 = -1.197
    c8 = 5.228
    c9 = -14.84
    c10 = 0
    c11 = 0
    c12 = -3.736E-3
    c13 = 3.891E-2
    c14 = 0
    c15 = 0
    c16 = 0.639
    c17 = 1.693

    Z = ZN / 1000  # convert to kN
    α = αr * 180 / π  # convert to degrees
    γ = γr * 180 / π  # convert to degrees

    Sh = c11 * γ + c12 * Z .+ c13
    Sv = (c14 * Z .^ 2 + c15 * Z) .* γ + c16 * Z .+ c17

    C = c0
    D = (c1 * Z .+ c2) .* Z
    B = (c3 * Z .+ c4) .* Z .* (1.0 .- c6 * abs.(γ)) .* exp.(-c5 * Z) / C ./ D
    Bslip = B .* (α + Sh)
    E = ((c7 * Z .+ c8) .* Z .+ c9) .* (1.0 .- c10 * abs.(γ))

    N = D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip))) + Sv

    -N
end
