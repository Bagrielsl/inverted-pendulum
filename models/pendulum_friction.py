
from numpy import sin, cos, tan, pi, sqrt

def pendulum_friction(var, t, U, par):
    m, l, I, g, c = par
    x, x_dot, theta, theta_dot = var
    u = U(t, var)
    dx = x_dot
    dtheta = theta_dot
    dx_dot = u/m
    dtheta_dot = l*(c*theta_dot - g*m*sin(theta) - u*cos(theta))/(-I + l**2*m)
    return [dx, dx_dot, dtheta, dtheta_dot]
