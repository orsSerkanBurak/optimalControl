import casadi as ca
import matplotlib.pyplot as plt
import numpy as np


# for people who cannot see an interactive plot, uncomment the following lines
import matplotlib
if matplotlib.get_backend() == 'agg':
    matplotlib.use('WebAgg')
print(f'backend: {matplotlib.get_backend()}')


T = 5                    # integration time

# state variables
x = ca.SX.sym('x', 1)         # state
lam = ca.SX.sym('lam', 1)     # costate
aug = ca.vertcat(x, lam)      # augmented state

# boundary values
x0bar = 1
lamTbar = -1

# optimal control
uopt = 0.5*x[0]

# state dynamics
xdot = x[0]*uopt-x[0]-uopt**2

# costate dynamics
lamdot = (-lam[0]*uopt)+lam[0];

# augmented dynamics
augdot = ca.vertcat(xdot, lamdot)

# integrator
# aim: create integrator function F(x_in, tf) that integrates the
# given dynamics starting at state x_in over a time interval tf and returns
# the resulting state.
tf = ca.SX.sym('tf')                  # integration interval

# create struct with information needed to define the integrator
# the dynamics are scaled by tf.
# this is because CVODES by default integrates over the time interval [0,1].
# integration of the scaled dynamics from 0 to 1 is equivalent to
# integration of the original unscaled dynamics from 0 to tf
dae = {'x': aug, 'p': tf, 'ode': tf * augdot}
# some options (precision, in the sense of absolut and relative error
# tolerance)
opts = {'abstol': 1e-8, 'reltol': 1e-8}
# create the desired integrator function
F = ca.integrator('F', 'cvodes', dae, opts)

## build dummy NLP

# initial value of lambda (to be found)
lam0 = ca.MX.sym('lam0', 1)

# compute augmented state at time T dependend on lam0
intout = F(x0=ca.vertcat(x0bar, lam0), p=T)
augT = intout["xf"]                      # augmented state at T 
lamT = intout["xf"][1:2]                 # final value lambda(T)

# terminal condition
# this residual should be zero
g = lamT - lamTbar
lbg = 0
ubg = 0

# NLP with dummy objective
nlp = {'x': lam0, 'f': 0, 'g': g}
solver = ca.nlpsol('solver', 'ipopt', nlp)
sol = solver(lbg=lbg, ubg=ubg)

lam0opt = sol["x"].full()

## simulate solution

# integrate in N timesteps to get intermediate results
N = 100
DT = T/N

# integration loop
AUG0 = np.vstack((x0bar, lam0opt)).flatten()
AUG = np.empty((AUG0.size, N+1))
AUG[:, 0] = AUG0
for i in range(N):
    intres = F(x0=AUG[:, i], p=DT)
    AUG[:, i+1] = intres["xf"].full().flatten()


# split int state, costate, controls
xopt = AUG[0, :]
lamopt = AUG[1, :]
u_opt = 0.5*xopt
# time grid
tvec = np.linspace(0, T, num=N+1)

# algebraic results
beta = 1
uoptA = 1 / (1 + beta * np.exp(tvec))
xoptA = 2 * uoptA
lamoptA = (-np.exp(5 * uoptA - 5)) * (np.exp((1 - uoptA) * tvec))

# error evaluation
errorU = u_opt - uoptA;
errorX = xopt - xoptA;
errorLam = lamopt - lamoptA;

plt.figure(1,dpi=1200)
plt.subplot(4,1,1)
plt.plot(tvec, xopt)
plt.plot(tvec, xoptA)
plt.legend([r'$y_{numerical}$',r'$y_{algebraic}$'],loc='upper right')
plt.xlabel(r'time, $t$')
plt.ylabel(r'state, $y$')
plt.grid(True)
plt.axis([0, 5,0, 1])

plt.subplot(4,1,2)
plt.plot(tvec, lamopt)
plt.plot(tvec, lamoptA)
plt.legend([r'$p_{numerical}$',r'$p_{algebraic}$'],loc='upper right')
plt.xlabel(r'time, $t$')
plt.ylabel(r'costate, $y$')
plt.grid(True)
plt.axis([0, 5,-1, 0.1])

plt.subplot(4,1,3)
plt.plot(tvec, u_opt)
plt.plot(tvec, uoptA)
plt.legend([r'$u_{numerical}$',r'$u_{algebraic}$'],loc='upper right')
plt.xlabel(r'time, $t$')
plt.ylabel(r'control, $y$')
plt.grid(True)
plt.axis([0, 5,0, 0.5])

plt.subplot(4,1,4)
plt.plot(tvec, errorX)
plt.plot(tvec, errorLam)
plt.plot(tvec, errorU)
plt.legend([r'$Error_{state}$',r'$Error_{costate}$',r'$Error_{control}$'],loc='upper right')
plt.xlabel(r'time, $t$')
plt.ylabel(r'error')
plt.grid(True)
plt.axis([0, 5,-0.11, 0.1])

plt.subplots_adjust(top=3,bottom=1,right=2)
plt.savefig('comparison.png', dpi=1200, bbox_inches='tight')
plt.show()
