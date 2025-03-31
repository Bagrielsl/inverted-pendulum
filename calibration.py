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
    
    def cost_function(b, I):
        # Unpack parameters
        M = 0.266 # kg
        m = 0.075 # kg
        #b = 0.1 # N/m/s
        l = 0.29 # m
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
    # Bounds for b and I
    b_bounds = (0.0, 100.0)
    I_bounds = (0.0, 1.0)
    # Optimize cost function
    result = minimize(lambda x: cost_function(x[0], x[1]), [b_init, I_init], bounds=[b_bounds, I_bounds])
    b_opt, I_opt = result.x
    
    # Print results
    print(f"Optimal b: {b_opt}")
    print(f"Optimal I: {I_opt}")
    print(f"Cost function value: {result.fun}")
    # Simulate with optimal parameters
    var0 = [0.0, 0.0, data_theta[0], 0.0]
    sol = odeint(pendulum, var0, data_t, args=(lambda t, x: 0.0, (0.266, 0.075, 0.29, I_opt, 9.78, b_opt)))
    # Plot results
    plt.plot(data_t, data_theta, 'r--', label='Data')
    plt.plot(data_t, np.abs(sol[:, 2]), 'b', label='Model')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration')
    plt.show()
calibrate_pendulum()
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
    
def show_test_soltar_out():
    data = pd.read_csv('data/test_soltar_out.csv')
    data_t = data['t']
    data_r = data['r']
    data_p = data['p']
    plt.plot(data_t, data_r, 'r--', label='Data r')
    plt.plot(data_t, data_p, 'b', label='Data p')
    plt.xlabel('Time (s)')
    plt.ylabel('Theta (rad)')
    plt.legend()
    plt.title('Pendulum Calibration Data')
    plt.show()
#show_test_soltar_out()