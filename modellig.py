import numpy as np
from scipy.integrate import odeint
import matplotlib.pyplot as plt
from sympy import solve, symbols, diff
import sympy as sp
from numpy import sin, cos, tan
import matplotlib.animation as animation
import control as clt


# Simple Pendulum Modelling

# generateing model
def generate_equations_pendulum(show=True):
    # parameters
    M, m, I, l, g, b = symbols('M m I l g b')
    # inputs 
    u, D = symbols('u D')
    
    # states
    theta, x, dtheta, dx = symbols('theta x theta_dot x_dot')
    ddx, ddtheta = symbols('ddx ddθ')
    eq1 = (M+m)*ddx - b*dx + m*l*ddtheta*sp.cos(theta)-m*l*dtheta**2*sp.sin(theta)-u
    eq2 = (I+m*l**2)*ddtheta+m*g*l*sp.sin(theta) + m*l*ddx*sp.cos(theta)
    sol = solve((eq1, eq2), (ddx, ddtheta))
    f1 = sp.simplify(sol[ddx])
    f2 = sp.simplify(sol[ddtheta])
    if show:
        print('dx_dot = ', f1)
        print('dtheta_dot = ', f2)
    return f1, f2


def pendulum(var, t, K, x_ref, par):
    M, m, l, I, g, b = par
    x, x_dot, theta, theta_dot = var
    
    #Lei de controle LQR
    state_error = np.array(var) - x_ref
    u = -np.dot(K, state_error)
    
    dx = x_dot
    dtheta = theta_dot
    dx_dot =  (I*l*m*theta_dot**2*sin(theta) + I*u + g*l**2*m**2*sin(2*theta)/2 + l**3*m**2*theta_dot**2*sin(theta) + l**2*m*u)/(I*M + I*m + M*l**2*m + l**2*m**2*sin(theta)**2)
    dtheta_dot =  -l*m*(M*g*sin(theta) + g*m*sin(theta) + l*m*theta_dot**2*sin(2*theta)/2 + u*cos(theta))/(I*M + I*m + M*l**2*m + l**2*m**2*sin(theta)**2)
    
    return [dx, dx_dot, dtheta, dtheta_dot]

def linearize_pendulum(vars_val, par_val = [0.5, 0.2, 0.3, 0.006, 9.8, 0.1]):
    x, x_dot, theta, theta_dot = symbols('x x_dot theta theta_dot')
    vars = [x, x_dot, theta, theta_dot]
    u = symbols('u')
    f1, f2 = generate_equations_pendulum(show=False)
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

def simulate(anim=False):
    # Parametros
    M = 0.5 # kg
    m = 0.2 # kg
    b = -0.1 #0.03 # N/m/s
    l = 0.3 # m
    I = 0.006 # kg m²
    g = 9.8 # m/s²

    K, E = lqr_control(Q=np.diag([1000, 1, 100, 1]), R=np.array([[100]]))
    x_ref = np.array([0.0, 0.0, np.pi, 0.0])
    
    par = M, m, l, I, g, b
    var0 = [0.0, 0.0, np.pi - 0.2, 0.0]

    t_start = 0
    t_end = 10
    t = np.linspace(t_start, t_end, 301)

    sol = odeint(pendulum, var0, t, args=(K.flatten(), x_ref, par))

    u_history = []
    for i in range(len(t)):
        state = sol[i]
        state_error = state - x_ref
        u = -np.dot(K, state_error)
        u_history.append(u)
    u_history = np.array(u_history)

    plt.figure(1)
    plt.plot(t, u_history, 'r', label='Força u(t)')
    plt.xlabel('Tempo (s)')
    plt.ylabel('Força (N)')
    plt.title('Força Aplicada no Carrinho')
    plt.grid()
    plt.legend()
    plt.show()
    plt.close()
    
    if not anim:
        plt.figure(2)
        plt.plot(t, sol[:, 0], 'b', label='x(t)')   
        plt.plot(t, sol[:, 2], 'g', label='theta(t)')
        plt.legend(loc='best')
        plt.show()
    else:
        x, theta = sol[:, 0], sol[:, 2]
        fig = plt.figure(3)
        lim = 1
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
        ani.save('pendulum.gif', writer='pillow', fps=30)
        

# conclusão não é estável
def open_loop_stability():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi/2, 0.0])
    sys = clt.ss(A, B, C, D)
    poles = clt.poles(sys)
    max_img = max([abs(p.imag) for p in poles])
    plt.vlines(0, -max_img, max_img)
    plt.plot(poles.real, poles.imag, 'x')
    plt.grid()
    plt.show()
    return poles

####################
# Controle Cascata
####################

## Loop interno

# transfer function
def system_tf():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    tf = clt.ss2tf(A, B, C, D)
    num1, num2 = tf.num
    den1, den2 = tf.den
    xu = clt.tf(num1[0], den1[0])
    thetau = clt.tf(num2[0], den2[0])
    print(xu, thetau)
    return xu, thetau
# system_tf()


# K proporcional loop interno
def Kp_inner_loop():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi/2, 0.0])
    sys = clt.ss(A, B, C, D)
    ks = np.linspace(-10, 10, 100)
    poles = []
    for k in ks:
        cls_loop = clt.feedback(clt.series(clt.tf([k], [1]), sys))
        poles.append(clt.pole(sys, k))

# Controllability
def controllability():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    mat_control = clt.ctrb(A, B)
    posto_control = np.linalg.matrix_rank(mat_control)
    return posto_control

####################
# LQR
####################

def lqr_control(Q, R):
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    sys = clt.ss(A, B, C, D)
    K, S, E = clt.lqr(sys, Q, R)
    print(K)
    return K, E

# simulate(anim=True)

def lqr_root_locus():
    A, B, _, _ = linearize_pendulum([0.0, 0.0, np.pi, 0.0])

    Q = np.eye(4)
    R_values = np.linspace(0.1, 100, 10)
    lqr_poles = []

    for R_val in R_values:
        R = np.array([[R_val]])
        _, E = lqr_control(Q, R)
        lqr_poles.append(E)

    lqr_poles = np.array(lqr_poles)

    plt.figure(figsize=(8,6))
    for i in range(lqr_poles.shape[1]):
        plt.plot(lqr_poles[:, i].real, lqr_poles[:, i].imag, '-o', label=f'Polos {i+1}')
    plt.xlabel('Real')
    plt.ylabel('Imaginário')
    plt.title('Root Locus LQR (Variando R)')
    plt.grid(True)
    plt.legend()
    plt.show()

def kalam_filter():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])

    Vd = .1*np.eye(4) # Disturbio covariance
    Vn = 1 # Ruido covariance

    BF = [B, Vd, 0*B]

    sysC = clt.ss(A, BF, C, [0, 0, 0, 0, 0, Vn])

print(linearize_pendulum([0.0, 0.0, np.pi, 0.0]))

Kf = np.array([[0.85583616, 0.31622777, 0.0, 0.0],
  [0.0, 0.0, 23.01547298, 264.8059983]])

A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
