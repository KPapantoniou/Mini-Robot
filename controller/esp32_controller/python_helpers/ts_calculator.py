import sympy as sp
import numpy as np

# Define symbolic variable
t = sp.Symbol('t', positive=True)  # Time we solve for
ts_val = 0.45                      # Example ts value you can adjust

# Define known constants
Kt = 0.0008373       # Torque constant
R = 10.71            # Motor resistance
b_damp = 2.61675e-08 # Damping coefficient
cf = 1.17222e-05     # Coulomb friction
J = 3.4375e-09       # Inertia
m = 0.0011           # Mass
g = 9.81             # Gravity
r = 0.0025           # Radius
theta = np.radians(0)  # 90 degrees in radians
u = 0             # Input signal
v_des = 1.5          # Max desired voltage
sign_u = 0           # Assume positive input

# Define the A term (static contribution)
A = ((Kt**2 + R*b_damp)/Kt)*u + (R*cf*sign_u)/Kt + R*m*g*r*sp.sin(theta)

# Controller output with ts fixed
output = -12*0/ts_val + 36*t*700/ts_val**2

# Full voltage expression
v_expr = A + (J*R/Kt) * output

# Solve v_expr = v_des for t
eq = sp.Eq(v_expr, v_des)
solution = sp.solve(eq, t)

# Display solution
print("Solutions for t:")
for sol in solution:
    if sol.is_real and sol >= 0:
        print(f"t = {sol.evalf()} seconds")
