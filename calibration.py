from models.pendulum_friction import pendulum_friction
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.integrate import odeint
import matplotlib.pyplot as plt
def calibrate_pendulum():
    data = pd.read_csv('data/test_1.csv')
    data_t = data['t']
    data_theta = data['p']/ 180 * np.pi
    
    c = -0.0035 # N/rad/s
    g = 9.78 # m/s²
    m = 0.075 # kg
    def cost_function(x):
        l, I = x
        # Unpack parameters
        #M = 0.266 # kg
        #b = 0.1 # N/m/s
        #l = 0.087 # m
        #I = 0.006 # kg m²
        par = m, l, I, g, c
        
        # Initialize pendulum model
        var0 = [0.0, 0.0, data_theta[0], 0.0]
        
        # Simulate pendulum
        sol = odeint(pendulum_friction, var0, data_t, args=(lambda t, x: 0.0, par))
        
        # Calculate cost function
        return np.mean((np.abs(sol[:, 2]) - data_theta)**2)
    
    # Initial guess for b and I
    l_init = 0.07
    I_init = 0.000001
    init = np.array([ l_init, I_init])
    # Bounds for b and I
    l_bounds = (0.01, 0.3)
    I_bounds = (0.0, 0.01)
    bounds = [l_bounds, I_bounds]
    # Optimize cost function
    result = minimize(cost_function, init, bounds=bounds, method='BFGS')
    l_opt, I_opt = result.x
    
    # Print results
    print(f"Optimal b: {l_opt}")
    print(f"Optimal I: {I_opt}")
    print(f"Cost function value: {result.fun}")
    # Simulate with optimal parameters
    var0 = [0.0, 0.0, data_theta[0], 0.0]
    par = m, l_opt, I_opt, g, c
    sol = odeint(pendulum_friction, var0, data_t, args=(lambda t, x: 0.0, par))
    # Plot results
    plt.plot(data_t, data_theta, 'r--', label='Data')
    plt.plot(data_t, np.abs(sol[:, 2]), 'b', label='Model')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration')
    plt.show()
#calibrate_pendulum()
def show_data_test_1():
    data = pd.read_csv('data/test_1.csv')
    data_t = data['t']
    data_theta = data['p']
    plt.plot(data_t, data_theta, 'r--', label='Data')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration Data')
    plt.show()
#show_data_test_1()
def show_test_soltar_out():
    data = pd.read_csv('data/test_soltar_out.csv')
    data_t = data['t']
    data_r = data['r']
    data_p = data['p']
    data_p = data_p.rolling(100).mean()
    d_p = data_p.diff()
    dd_p = d_p.diff()
    ch = (np.abs(d_p) < 0.01) & (dd_p < 0.0)
    plt.plot(data_t, data_r, 'r--', label='Data r')
    plt.plot(data_t, data_p, 'b', label='Data p')
    plt.plot(data_t, d_p, 'g', label='Data p dot')
    plt.plot(data_t, 10*dd_p, 'y', label='Data p double dot')
    plt.plot(data_t, data['p']*0, 'k', label='Zero')
    plt.plot(data_t, 10*ch, 'k--', label='virada')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration Data')
    plt.show()
#show_test_soltar_out()

def raw_pitch():
    code = None
    with open('data/test_soltar.txt', 'r') as file:
        code = file.read()
    p = []
    r = []
    for line in code.split('\n')[1:]:
        sline = line.split()
        try:
            r.append(float(sline[3]))
            p.append(float(sline[5]))
        except:
            print('error:',line)
    
    p = np.array(p)
    r = np.array(r)
    for i in range(len(p)):
        if r[i] < 0:
            p[i] = -p[i] + 180
    plt.plot(p)
    plt.plot(r)
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.title('Pendulum Calibration Data')
    plt.show()
#raw_pitch()


def simul_test():
    
    data = pd.read_csv('data/test_1.csv')
    data_t = data['t']
    data_theta = data['p']/ 180 * np.pi
    # Simulate with optimal parameters
    var0 = [0.0, 0.0, data_theta[0], 0.0]
    
    # Unpack parameters
    m = 0.075 # kg
    l = 0.07 # m
    I = 0.000001 # kg m²
    g = 9.78 # m/s²
    c = -0.0035 # N/rad/s
    par = m, l, I, g, c
    
    
    sol = odeint(pendulum_friction, var0, data_t, args=(lambda t, x: 0.0, par))
    # Plot results
    plt.plot(data_t, data_theta, 'r--', label='Data')
    plt.plot(data_t, np.abs(sol[:, 2]), 'b', label='Model')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration')
    plt.show()
    
simul_test()
