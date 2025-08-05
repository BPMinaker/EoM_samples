function driver(l, offset, heading, x)

    # get the road location, heading
    offset_t, heading_t, curvature = track(x)

    # find the error in location and heading
    offset_error = offset_t - offset
    heading_error = heading_t - heading

    # compute the appropriate steer angle to return to the road
    l * curvature + 1.1 * heading_error + 0.1 * offset_error
end