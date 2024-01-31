import matplotlib.pyplot as plt
import numpy as np
import sys
import os.path
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
from Vehicle.sensors import *


def create_frequency_distribution(data, bins):
    frequency_distribution, bin_edges = np.histogram(data, bins=bins)
    return dict(zip(bin_edges[:-1], frequency_distribution))

# Example usage:
data_list = []
truth = [10]
sensor = Accelerometer()
for i in range(10000):
    data_list.append(sensor.reading(truth))
bins = np.linspace(min(data_list)[0], max(data_list)[0] + 1, 200)  # You can adjust the bin width as needed
result = create_frequency_distribution(data_list, bins)

# Plot the frequency distribution
plt.bar(result.keys(), result.values(), width=0.8*(bins[1] - bins[0]))
plt.xlabel('Buckets')
plt.ylabel('Frequency')
plt.title('Frequency Distribution with Bins')
plt.show()