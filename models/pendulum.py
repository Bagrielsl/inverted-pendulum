
from numpy import sin, cos, tan

def pendulum(var, t, U, par):
    M, m, l, I, g, b = par
    x, x_dot, theta, theta_dot = var
    u = U(t, var)
    dx = x_dot
    dtheta = theta_dot
    dx_dot = (-I*b*x_dot + I*l*m*theta_dot**2*sin(theta) + I*u - b*l**2*m*x_dot + g*l**2*m**2*sin(2*theta)/2 + l**3*m**2*theta_dot**2*sin(theta) + l**2*m*u)/(I*M + I*m + M*l**2*m + l**2*m**2*sin(theta)**2)
    dtheta_dot = l*m*(-M*g*sin(theta) + b*x_dot*cos(theta) - g*m*sin(theta) - l*m*theta_dot**2*sin(2*theta)/2 - u*cos(theta))/(I*M + I*m + M*l**2*m + l**2*m**2*sin(theta)**2)
    return [dx, dx_dot, dtheta, dtheta_dot]
