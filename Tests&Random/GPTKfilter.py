import control as ct
from control import *
import matplotlib.pyplot as plt

G = ct.ss([[-1, -2], [3, -4]], [[5], [7]], [[6, 8]], [[9]])
plt.figure()
ct.bode_plot(G)
plt.show()

