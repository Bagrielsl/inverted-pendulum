import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from sympy import solve, symbols, diff
import sympy as sp
from numpy import sin, cos, tan
import matplotlib.animation as animation
from models.pendulum import pendulum
import control as clt


# Simple Pendulum Modelling

# generateing model
def generate_equations_pendulum():
    # parameters
    M, m, I, l, g, b = symbols('M m I l g b')
    # inputs 
    u, D = symbols('u D')
    # states
    theta, x, dtheta, dx = symbols('theta x theta_dot x_dot')
    ddx, ddtheta = symbols('ddx ddθ')
    eq1 = (M+m)*ddx + b*dx + m*l*ddtheta*sp.cos(theta)-m*l*dtheta**2*sp.sin(theta)-u
    eq2 = (I+m*l**2)*ddtheta+m*g*l*sp.sin(theta) + m*l*ddx*sp.cos(theta)
    sol = solve((eq1, eq2), (ddx, ddtheta))
    f1 = sp.simplify(sol[ddx])
    f2 = sp.simplify(sol[ddtheta])
    return f1, f2

def generate_equations_pendulum_file():
    # parameters
    f1, f2 = generate_equations_pendulum()
    sep = '\n    '
    value = f'dx = x_dot{sep}dtheta = theta_dot{sep}'
    value += f'dx_dot = {f1}{sep}dtheta_dot = {f2}'
    template = ''
    with open('models/template_model.txt', 'r') as f:
        template = f.read()
    func = template.replace('@@', value)
    with open('models/pendulum.py', 'w') as f:
        f.write(func)
    print('new pendulum model generated')


def linearize_pendulum(vars_val, par_val = [0.5, 0.2, 0.3, 0.006, 9.8, 0.1]):
    x, x_dot, theta, theta_dot = symbols('x x_dot theta theta_dot')
    vars = [x, x_dot, theta, theta_dot]
    u = symbols('u')
    f1, f2 = generate_equations_pendulum()
    eqs = [x_dot, f1, theta_dot, f2]
    A = np.zeros((4, 4)).astype('object')
    B = np.zeros((4, 1)).astype('object')
    for i, eq in enumerate(eqs):
        for j, var in enumerate(vars):
            A[i, j] = diff(eq, var)
        B[i] = diff(eq, u)
    par = symbols('M m l I g b')
    
    for i in range(4):
        for j in range(4):
            A[i, j] = A[i, j].subs(list(zip(par, par_val)))
            A[i, j] = A[i, j].subs(list(zip(vars, vars_val)))
            A[i, j] = A[i, j].subs(u, 0.0)
        B[i, 0] = B[i, 0].subs(list(zip(par, par_val)))
        B[i, 0] = B[i, 0].subs(list(zip(vars, vars_val)))
        B[i, 0] = B[i, 0].subs(u, 0.0)
    C = np.array([[1, 0, 0, 0], [0, 0, 1, 0]])
    D = np.zeros((2, 1))
    return A.astype('float'), B.astype('float'), C, D

def simulate(anim=False, u=lambda t, x : 0.0 , var0=[0.0, 0.0, np.pi*0.98, 0.0], t_start=0, t_end=5, lim=3, save=False):
    # Parametros
    M = 0.5 # kg
    m = 0.2 # kg
    b = 0.1 # N/m/s
    l = 0.3 # m
    I = 0.006 # kg m²
    g = 9.8 # m/s²
    par = M, m, l, I, g, b

    t_start = 0
    t_end = 5
    t = np.linspace(t_start, t_end, 301)

    sol = odeint(pendulum, var0, t, args=(u, par))
    
    if not anim:
        plt.plot(t, sol[:, 0], 'b', label='x(t)')
        plt.plot(t, sol[:, 2], 'g', label='theta(t)')
        plt.legend(loc='best')
        plt.show()
    else:
        x, theta = sol[:, 0], sol[:, 2]
        fig = plt.figure()
        ax = fig.add_subplot(autoscale_on=False, xlim=(-lim, lim), ylim=(-lim, lim))
        ax.set_aspect('equal')
        ax.grid()
        
        cm_car, = ax.plot([], [], 's-', markersize=M*20)
        rod, = ax.plot([], [], 'o-', lw=2, markersize=m/l)
        trajectory, = ax.plot([], [], 'r', lw=0.1, linestyle='--')
        def init():
            cm_car.set_data([], [])
            rod.set_data([], [])
            trajectory.set_data([], [])
            return cm_car, rod, trajectory
        
        def animate(i):
            cm_car.set_data([x[i]], [0.0])
            rod.set_data([x[i], x[i]+l*sin(theta[i])], [0.0,-l*cos(theta[i])])
            trajectory.set_data(x[:i]+l*sin(theta[:i]), -l*cos(theta[:i]))
            return cm_car, rod, trajectory
        
        interval = (t_end - t_start) / len(t)*1000
        ani = animation.FuncAnimation(fig, animate, init_func=init, frames=len(t),
                                      interval=interval, blit=True)
        
        plt.show()
        if save:
            ani.save('pendulum.gif', fps=30)

def simulate_u(u=lambda t, x : 0.0 , var0=[0.0, 0.0, np.pi*0.98, 0.0], t_start=0, t_end=5, save=False):
    # Parametros
    M = 0.5 # kg
    m = 0.2 # kg
    b = 0.1 # N/m/s
    l = 0.3 # m
    I = 0.006 # kg m²
    g = 9.8 # m/s²
    par = M, m, l, I, g, b

    t_start = 0
    t_end = 5
    t = np.linspace(t_start, t_end, 301)

    sol = odeint(pendulum, var0, t, args=(u, par))
    sol_u = [u(t, x) for t, x in zip(t, sol)]
    plt.plot(t, sol_u, 'b', label='u(t)')
    plt.legend(loc='best')
    plt.show()
    if save:
        plt.savefig('u.png')
    
def compare_nonlinear_linear(point=[0.0, 0.0, np.pi, 0.0]):
    A, B, C, D = linearize_pendulum(point)
    sys = clt.ss(A, B, C, D)
    t = np.linspace(0, 1, 301)
    _, sol_lin = clt.initial_response(sys, T=t)
    x_lin, theta_lin = sol_lin[0].flatten(), sol_lin[1].flatten()
    x_lin, theta_lin = x_lin + point[0], theta_lin + point[2]
    par = [0.5, 0.2, 0.3, 0.006, 9.8, 0.1]
    u = lambda t, x : 0.0
    sol = odeint(pendulum, point, t, args=(u, par))
    x, theta = sol[:, 0], sol[:, 2]
    print(x.shape, x_lin.shape)
    fig, ax = plt.subplots()
    ax.plot(t, x, 'b', label='x(t)')
    ax.plot(t, theta, 'g', label='theta(t)')
    ax.plot(t, x_lin, 'b', ls='--', label='x_lin(t)')
    ax.plot(t, theta_lin, 'g', ls='--', label='theta_lin(t)')
    ax.legend(loc='best')
    plt.show()
    
#compare_nonlinear_linear(point=[0.0, 0.0, np.pi*0.98, 0.0])

# Pendulum with Friction in the rod
# This model dont modeling the car equations just accpet the input u as a speed const speed times [0, 1, -1]
# and the friction is in the rod.
def generate_equations_pendulum_friction():
    # parameters
    M, m, I, l, g, b = symbols('M m I l g b')
    # inputs 
    u, D = symbols('u D')
    # states
    theta, x, dtheta, dx = symbols('theta x theta_dot x_dot')
    ddx, ddtheta = symbols('ddx ddθ')
    eq1 = (M+m)*ddx + b*dx + m*l*ddtheta*sp.cos(theta)-m*l*dtheta**2*sp.sin(theta)-u
    eq2 = (I+m*l**2)*ddtheta+m*g*l*sp.sin(theta) + m*l*ddx*sp.cos(theta) - b*l*dtheta
    sol = solve((eq1, eq2), (ddx, ddtheta))
    f1 = sp.simplify(sol[ddx])
    f2 = sp.simplify(sol[ddtheta])
    return f1, f2