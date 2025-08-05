# magic formula tire model parameters
mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / π, 6494.6, 4.7966E-3 * 180 / π, -0.3875E-3, 1.0]

# define a nonlinear tire with load sensitivity
function tire(Z, slip)
    C = mtm[1]
    D = (mtm[2] * Z .+ mtm[3]) .* Z
    B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
    Bslip = B .* slip
    E = mtm[7] * Z .+ mtm[8]

    -D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
end



