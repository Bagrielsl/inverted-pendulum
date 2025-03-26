import numpy as np
import matplotlib.pyplot as plt
from modelling import linearize_pendulum, simulate, simulate_u
import control as clt


# conclusão não é estável
def open_loop_stability():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    sys = clt.ss(A, B, C, D)
    poles = clt.poles(sys)
    max_img = max([abs(p.imag) for p in poles])
    plt.vlines(0, -max_img, max_img)
    plt.plot(poles.real, poles.imag, 'x')
    plt.grid()
    plt.show()
    return poles

############
# LQR
############

def system_lqr():
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    Q = np.eye(4)
    Q[0, 0] = 10
    R = 1000*np.eye(1)
    K, _, _ = clt.lqr(A, B, Q, R)
    K = K[0]
    print(f'[{K[0]}, {K[1]}, {K[2]}, {K[3]}]')
    return K

def simulate_lqr():
    K = system_lqr()
    def u(t, x):
        error = x - [0.0, 0.0, np.pi, 0.0]
        return -np.dot(K, error)
    var0 = [0.0, 0.0, np.pi -0.2, 0.0]
    simulate(anim=True, u=u, var0=var0, lim=1, save=False)
    simulate_u(u, var0, save=False)
simulate_lqr()

def double_lqr():
    Q = np.eye(4)
    Q[0, 0] = 10
    R = 100*np.eye(1)
    
    A, B, C, D = linearize_pendulum([0.0, 0.0, np.pi, 0.0])
    K, _, _ = clt.lqr(A, B, Q, R)
    K1 = K[0]
    
    R = 0.01*np.eye(1)
    A, B, C, D = linearize_pendulum([0.0, 0.0, 0.0, 0.0])
    K, _, _ = clt.lqr(A, B, Q, R)
    K2 = K[0]
    return K1, K2

def simulate_double_lqr():
    K1, K2 = double_lqr()
    def u(t, x):
        error = x - [0.0, 0.0, np.pi, 0.0]
        if abs(x[2]) < np.pi/2:
            return -np.dot(K2, error)
        return -np.dot(K1, error)
    var0 = [0.0, 0.0, np.pi*0.5, 0.0]
    simulate(anim=True, u=u, var0=var0, lim=10, save=False)
    simulate_u(u, var0, save=False)
    
#simulate_double_lqr()

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
    return xu, thetau




def Ki_inner_loop():
    xu, thetau = system_tf()
    kp = Kp_inner_loop()
    ks = np.linspace(-100, 100, 1000)
    s = clt.TransferFunction.s
    ys = []
    for k in ks:
        sys = clt.feedback((kp + k/s)*xu)
        T, y = clt.step_response(sys)
        ys.append(y[-1])
    my = np.argmin(np.abs(ys))
    print('Ki = ', ks[my], 'y = ', ys[my], my)
    plt.plot(ks, ys)
    plt.show()
    return ks[my]

#Ki_inner_loop()

# K derivativo loop interno

def Kd_inner_loop():
    xu, _ = system_tf()
    kp = Kp_inner_loop()
    ki = Ki_inner_loop()
    ks = np.linspace(0, 10, 1000)
    s = clt.TransferFunction.s
    ys = []
    for k in ks:
        sys = clt.feedback(kp*xu + ki/s + k*s)
        T, y = clt.step_response(sys)
        ys.append(y[-1])
    my = np.argmin(np.abs(ys))
    print('Kd = ', ks[my], 'y = ', ys[my])
#Kd_inner_loop()

# simulate(anim=True)
