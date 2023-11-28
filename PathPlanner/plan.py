import numpy as np
from scipy.optimize import linprog
import matplotlib.pyplot as plt
import Graphing.plotter

H_value_updated = 50  # New maximum height

# Recalculate the height values with the updated maximum height
h1_values_updated = np.array([-16*H_value_updated/T_value**3 * t**3 + 12*H_value_updated/T_value**2 * t**2 for t in t_ascent])
h2_values_updated = np.array([16*H_value_updated/T_value**3 * t**3 - 36*H_value_updated/T_value**2 * t**2 + 24*H_value_updated/T_value * t - 4*H_value_updated for t in t_descent])

# Combine the ascent and descent for plotting
h_combined_updated = np.concatenate([h1_values_updated, h2_values_updated])

# Plotting the updated trajectory
plt.figure(figsize=(10, 6))
plt.plot(t_combined, h_combined_updated, label='Height vs Time')
plt.title('Cubic Ascent and Descent Trajectory (50m Height)')
plt.xlabel('Time (s)')
plt.ylabel('Height (m)')
plt.grid(True)
plt.legend()
plt.show()
    
