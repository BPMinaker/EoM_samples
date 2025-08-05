using ForwardDiff

function track(x)
    # define road y coordinate using the EoM function `pulse()` to paste together a piecewise function of sines and constant; pulse(x, a, b) = 1 if a<x<b, 0 otherwise
    # this defines a value of y=0 for x<50, y ramps from 0 to 4 for 50<x<100, y=4 for 100<x<150, y ramps from 4 back to 0 for 150<x<200 and y=0 for x>200
    y(x) = 4 * (EoM.pulse(x, 50, 100) * ((x - 50) / 50 - sin(2π * (x - 50) / 50) / 2π) + EoM.pulse(x, 100, 150) + EoM.pulse(x, 150, 200) * (1 - (x - 150) / 50 + sin(2π * (x - 150) / 50) / 2π))

    # use automatic differentiation to find the heading angle and curvature; as long as the angles are small we can approximate heading with the slope found with the derivative and the curvature as the second derivative; automatic differentiation is a powerful numerical (i.e., not symbolic!) technique to compute the derivative of any function, using the fact that every function must be computed numerically using basic arithmetic operations (+-*/), and that the derivative of these operations is well defined
    dy(x) = ForwardDiff.derivative(y, x)
    d2y(x) = ForwardDiff.derivative(dy, x)

    # evaluate all three and return those values
    y(x), dy(x), d2y(x)
end