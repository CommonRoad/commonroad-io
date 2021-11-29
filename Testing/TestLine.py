# Importing packages
import matplotlib.pyplot as plt

# Define x and y values
x = [7, 14, 21, 28, 35, 42, 49]
y = [8, 13, 21, 30, 31, 44, 50]

# Plot a simple line chart with 'solid' linestyle
plt.plot(x, y, linestyle='-')
plt.show()

# Plot a simple line chart with 'dashed' linestyle
plt.plot(x, y, linestyle='dashed')
# Or you can use: plt.plot(x, y, linestyle='--')
plt.show()

# Plot a simple line chart with 'dotted' linestyle
plt.plot(x, y, ':')
plt.show()

# Plot a simple line chart with 'dash_dot' linestyle
plt.plot(x, y, '-.')
plt.show()