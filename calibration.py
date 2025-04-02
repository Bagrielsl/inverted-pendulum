from models.pendulum import pendulum
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from scipy.integrate import odeint
import matplotlib.pyplot as plt
def calibrate_pendulum():
    data = pd.read_csv('data/test_1.csv')
    data_t = data['t']
    data_theta = data['p']/ 180 * np.pi
    
    def cost_function(x):
        b, I, l, M = x
        # Unpack parameters
        #M = 0.266 # kg
        m = 0.075 # kg
        #b = 0.1 # N/m/s
        #l = 0.29 # m
        #I = 0.006 # kg m²
        g = 9.78 # m/s²
        par = M, m, l, I, g, b
        
        # Initialize pendulum model
        var0 = [0.0, 0.0, data_theta[0], 0.0]
        
        # Simulate pendulum
        sol = odeint(pendulum, var0, data_t, args=(lambda t, x: 0.0, par))
        
        # Calculate cost function
        return np.mean((np.abs(sol[:, 2]) - data_theta)**2)
    
    # Initial guess for b and I
    b_init = 0.1
    I_init = 0.006
    l_init = 0.12
    M_init = 10#0.266
    init = np.array([b_init, I_init, l_init, M_init])
    # Bounds for b and I
    b_bounds = (0.0, 100.0)
    I_bounds = (0.0, 1.0)
    l_bounds = (0.0, 0.15)
    M_bounds = (0.01,100.0)#(0.001, 100.0)
    bounds = [b_bounds, I_bounds, l_bounds, M_bounds]
    # Optimize cost function
    result = minimize(cost_function, init, bounds=bounds, method='BFGS')
    b_opt, I_opt, l_opt, M_opt = result.x
    
    # Print results
    print(f"Optimal b: {b_opt}")
    print(f"Optimal I: {I_opt}")
    print(f"Optimal l: {l_opt}")
    print(f"Optimal M: {M_opt}")
    print(f"Cost function value: {result.fun}")
    # Simulate with optimal parameters
    var0 = [0.0, 0.0, data_theta[0], 0.0]
    par = M_opt, 0.075, l_opt, I_opt, 9.78, b_opt
    sol = odeint(pendulum, var0, data_t, args=(lambda t, x: 0.0, par))
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
show_test_soltar_out()