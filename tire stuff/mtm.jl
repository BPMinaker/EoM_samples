mtm = [1.6929, -55.2084E-6, 1.27128, 1601.8 * 180 / pi, 6494.6, 4.7966E-3 * 180 / pi, -0.3875E-3, 1.0]

# define a nonlinear tire with load sensitivity
function tire(Z, slip)
    C = mtm[1]
    D = (mtm[2] * Z .+ mtm[3]) .* Z
    B = mtm[4] * sin.(2 * atan.(Z / mtm[5])) / C ./ D
    Bslip = B .* slip
    E = mtm[7] * Z .+ mtm[8]

    D .* sin.(C * atan.((1.0 .- E) .* Bslip + E .* atan.(Bslip)))
end

using Plots
plotly()


Zrange = 100:200:6000
sliprange = 0:0.01:0.3

F = [tire(Z, slip) for Z in Zrange, slip in sliprange]

using LinearAlgebra
U, S, V = svd(F)
# The best rank-1 approximation is S[1] * U[:,1] * V[:,1]'

s1 = sqrt(S[1])
fZ_approx = U[:,1] * s1 * 65
gα_approx = V[:,1]' * s1 / 65

Fapprox = fZ_approx * gα_approx


plot(sliprange, Zrange, F; st=:surface, size=(800, 600), xlabel="α", ylabel="Z", zlabel="F")
display(plot!(sliprange, Zrange, Fapprox; st=:surface))

display(plot(sliprange, Zrange, (F - Fapprox)./F ; st=:surface, size=(800, 600)))


using LsqFit


@. Zmodel(x, p) = (p[1]*x + p[2]*x^2)
p0 = [1.3, -1.e-6]

fit = curve_fit(Zmodel, Zrange, fZ_approx, p0)
pZ = fit.param

display(plot(Zrange, [fZ_approx, Zmodel.(Zrange, [fit.param])], label=["data" "fit"], size=(800, 600)))


@. slipmodel(x, p) = (p[1]*x + p[2]*p[3]*x^2)/(1 + p[2]*x^2)
p0 = [18., 134., 0.34]

fit = curve_fit(slipmodel, sliprange, gα_approx', p0)
pα = fit.param

display(plot(sliprange, [gα_approx' slipmodel.(sliprange, [fit.param])], label=["data" "fit"], size=(800, 600)))



function tire_approx(Z, slip)
    fZ = Zmodel(Z, pZ)
    gα = slipmodel(slip, pα)
    return fZ * gα
end


FF = [tire_approx(Z, slip) for Z in Zrange, slip in sliprange]


plot(sliprange, Zrange, F; st=:surface, size=(800, 600), xlabel="α", ylabel="Z", zlabel="F")
display(plot!(sliprange, Zrange, FF; st=:surface))

display(plot(sliprange, Zrange, (F - FF)./F ; st=:surface, size=(800, 600)))








