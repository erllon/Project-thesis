# See https://se.mathworks.com/help/optim/ug/fmincon.html for results using MATLAB fmincon()
# Documentation on scipy.optimize.minimize() https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html

# minimize returns:
#     fun: value of objective function at solution
#     jac: value of at jacobian of objective function at solution
#     message: Description of the cause of termination
#     nfev: Number of evaluations of the objective function
#     nit: Number of iterations performed by the optimizer
#     njev: Number of evaluations of the jacobian of the objective function


# %% imports...
import numpy as np
from scipy.optimize import Bounds
from scipy.optimize import LinearConstraint
from scipy.optimize import minimize

def fun_constr(x) -> float:
    return 100*(x[1]-x[0]**2)**2+(1-x[0])**2

def fun_constr_jac(x) -> float:
    return np.array([-400*(x[1]-x[0]**2)*x[0]-2*(1-x[0]),
                    200*(x[1]-x[0]**2)])

def fun_bounds(x) -> float:
    return 1+x[0]/(1+x[1]) - 3*x[0]*x[1]+ x[1]*(1+x[0])

def ineq_con1(x, A, b):
    #return 1 - x[0]-2*x[1] # -x[0]+2*x[1] <= 1
    return b - A@x
def eq1_con(x, Aeq, beq):
    #return 2*x[0]+x[1] - 1
    return Aeq@x - b

def circlecon(x):
    return -(x[0]-1/3)**2 - (x[1]-1/3)**2 + (1/3)**2 #circle with radius 1/3 centered at (1/3, 1/3)

#%% Linear constraints
x0 = np.array([0.5,0]).reshape(2,1)
A= np.array([1,2])
b = 1
Aeq = np.array([2,1])
beq = 1

lin_cons = [{'type':'ineq', 'fun': lambda x, A, b: b - A@x,'args':(A,b)} ,
        {'type':'eq', 'fun': lambda x, Aeq, beq: Aeq@x - b, 'args':(Aeq,beq)}]
# cons = [{'type':'ineq', 'fun': ineq_con1},
#         {'type':'eq', 'fun': eq1_con}]

res1 = minimize(fun_constr,x0,constraints=lin_cons)
print(f"*********res1:**********\n {res1}")

#%% Bounds
lb = np.array([0,0]).reshape(2,1)
ub = np.array([1,2]).reshape(2,1)

x0 = (lb+ub)/2

res2 = minimize(fun_bounds,x0, bounds=Bounds(lb,ub))

print(f"*********res2:**********\n {res2}")

# %% Nonlinear constraints
lb = np.array([0, 0.2]).reshape(2,1)
ub = np.array([0.5, 0.8]).reshape(2,1)

x0 = np.array([1/4, 1/4]).reshape(2,1)

nonlin_cons = {'type':'ineq', 'fun': circlecon}

res3 = minimize(fun_constr, x0, bounds=Bounds(lb,ub), constraints=nonlin_cons)

print(f"*********res3:**********\n {res3}")
# %% include jacobian
lb = np.array([-2,-2]).reshape(2,1)
ub = np.array([2,2]).reshape(2,1)
x0 = np.array([-1,2]).reshape(2,1)

res4 = minimize(fun_constr, x0, jac=fun_constr_jac,bounds=Bounds(lb,ub))

print(f"*********res4:**********\n {res4}")