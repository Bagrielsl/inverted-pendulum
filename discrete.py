import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from sympy import symbols, solve, diff
import sympy as sp
import control as ctrl
from numpy import sin, cos

# Assume que pendulum.py existe
from models.pendulum import pendulum

# Funções existentes (mantidas iguais, mas omitidas por brevidade)
def generate_equations_pendulum():
    M, m, I, l, g, b = symbols('M m I l g b')
    u, D = symbols('u D')
    theta, x, dtheta, dx = symbols('theta x theta_dot x_dot')
    ddx, ddtheta = symbols('ddx ddθ')
    eq1 = (M+m)*ddx + b*dx + m*l*ddtheta*sp.cos(theta) - m*l*dtheta**2*sp.sin(theta) - u
    eq2 = (I+m*l**2)*ddtheta + m*g*l*sp.sin(theta) + m*l*ddx*sp.cos(theta)
    sol = solve((eq1, eq2), (ddx, ddtheta))
    f1 = sp.simplify(sol[ddx])
    f2 = sp.simplify(sol[ddtheta])
    return f1, f2

def linearize_pendulum(vars_val, par_val=[0.5, 0.2, 0.3, 0.006, 9.8, 0.1]):
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

def system_lqr_discrete(Ts=0.01):
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    sys_continuous = ctrl.ss(A, B, C, D)
    sys_discrete = ctrl.c2d(sys_continuous, Ts, method='zoh')
    Ad = sys_discrete.A
    Bd = sys_discrete.B
    Q = np.eye(4)
    Q[0, 0] = 100
    # Q[2, 2] = 100
    R = np.array([[10]])
    K, _, _ = ctrl.dlqr(Ad, Bd, Q, R)
    print(f"Ganho LQR discretizado K: {K}")
    return K, Ad, Bd

def simulate_lqr_discrete():
    Ts = 0.01
    K, Ad, Bd = system_lqr_discrete(Ts)
    t_start = 0
    t_end = 5
    t = np.arange(t_start, t_end, Ts)
    n_steps = len(t)
    
    var0 = np.array([0.0, 0.0, np.pi-1, 0.0])
    x_ref = np.array([0.0, 0.0, np.pi, 0.0])
    
    x_history = np.zeros((4, n_steps))
    u_history = np.zeros(n_steps)
    x = var0.copy()
    
    par = (0.5, 0.2, 0.3, 0.006, 9.8, 0.1)
    
    for i in range(n_steps):
        x_history[:, i] = x
        error = x - x_ref
        u = -K @ error
        u_scalar = float(u[0])
        u_history[i] = u_scalar
        t_step = [t[i], t[i] + Ts]
        sol = odeint(pendulum, x, t_step, args=(lambda t, x: u_scalar, par))
        x = sol[-1]
    
    # Animação
    l = 0.3
    M = 0.5
    m = 0.2
    
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(autoscale_on=False, xlim=(-1, 1), ylim=(-0.5, 1))
    ax.set_aspect('equal')
    ax.grid()
    
    cm_car, = ax.plot([], [], 's-', markersize=M*20, label='Carro')
    rod, = ax.plot([], [], 'o-', lw=2, markersize=m/l, label='Pêndulo')
    trajectory, = ax.plot([], [], 'r--', lw=0.5, label='Trajetória')
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
    ax.legend(loc='upper right')
    
    def init():
        cm_car.set_data([], [])
        rod.set_data([], [])
        trajectory.set_data([], [])
        time_text.set_text('')
        return cm_car, rod, trajectory, time_text
    
    def animate(i):
        cm_car.set_data([x_history[0, i]], [0.0])
        theta = x_history[2, i]
        rod.set_data([x_history[0, i], x_history[0, i] + l*sin(theta)], 
                     [0.0, -l*cos(theta)])
        trajectory.set_data(x_history[0, :i+1] + l*sin(x_history[2, :i+1]), 
                           -l*cos(x_history[2, :i+1]))
        time_text.set_text(f'Tempo: {t[i]:.2f}s')
        return cm_car, rod, trajectory, time_text
    
    ani = animation.FuncAnimation(fig, animate, init_func=init, frames=n_steps,
                                  interval=Ts*1000, blit=True)
    plt.title('Pêndulo Invertido com LQR Discretizado')
    plt.xlabel('Posição (m)')
    plt.ylabel('Altura (m)')
    plt.show()
    
    # Gráfico da ação de controle discreta
    plt.figure(figsize=(10, 4))
    plt.step(t, u_history, 'r', where='post', label='u(t) discreto')
    plt.grid(True)
    plt.legend()
    plt.title('Ação de Controle Discreta')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Força (N)')
    plt.show()
    
    # Gráfico dos estados para verificação
    plt.figure(figsize=(10, 6))
    plt.plot(t, x_history[0, :], label='x (carro)')
    plt.plot(t, x_history[2, :] - np.pi, label='theta - π')
    plt.grid(True)
    plt.legend()
    plt.title('Estados do Sistema')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Magnitude')
    plt.show()

simulate_lqr_discrete()