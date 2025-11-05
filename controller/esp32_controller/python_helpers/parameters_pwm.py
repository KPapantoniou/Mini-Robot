import numpy as np

# Given data points
voltages = np.array([1.48, 1.33, 1.04, 0.71])
duties = np.array([98, 84, 56, 28])

# Build the matrix A and vector y for the system: A * [a, b, c] = y
A = np.vstack([voltages**3,voltages**2, voltages, np.ones_like(voltages)]).T
y = duties

# Solve for [a, b, c]
coeffs = np.linalg.solve(A, y)
a, b, c, d = coeffs

print(f"a = {a:.6f}")
print(f"b = {b:.6f}")
print(f"c = {c:.6f}")
print(f"d = {d:.6f}")
