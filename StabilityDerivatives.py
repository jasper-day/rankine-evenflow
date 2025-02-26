# Calculate the stability derivatives for a given aircraft
# Uses fourth-order accurate derivative calculation
# stability derivatives of interest
# CD0 - Zero lift drag coefficient
# CDalpha - change in drag coefficient with angle of attack
# CDu - change in drag coefficient with velocity
# CL0 - Lift coefficient
# CLalpha - Lift slope
# CLq - change in lift with roll rate
# CLu - change in lift with velocity
# Cm0 - Yaw moment coefficient
# Cmalpha - change in yaw moment with angle of attack
# Cmq - change in yaw moment with pitch rate
# Clbeta - Change in roll moment with sideslip
# Clp - Change in roll moment with roll rate
# Clr - Change in roll moment with pitch rate
# CYbeta - Change in side force with sideslip
# CYp - change in side force with roll rate
# CYr - change in side force with pitch rate
# Cnbeta - change in pitch moment with sideslip
# Cnp - change in pitch moment with roll rate
# Cnr - change in pitch moment with pitch rate

import jsbsim
import numpy as np
from tabulate import tabulate

# Load the aircraft model
AIRCRAFT_NAME = "EvenFlow"
PATH_TO_JSBSIM_FILES = "."
# Avoid flooding the console with log messages
jsbsim.FGJSBBase().debug_lvl = 0
fdm = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)
fdm.load_model(AIRCRAFT_NAME)

# trim condition
# steady level flight

# Initial conditions
u0 = 30
fdm["ic/h-sl-ft"] = 1000
fdm["ic/vc-kts"] = u0
fdm["ic/gamma-deg"] = 0
fdm["ic/beta-deg"] = 0

# Initialize the aircraft with initial conditions
fdm["propulsion/engine/set-running"] = 1
fdm.run_ic()

# Full trim
jsbsim.FGJSBBase().debug_lvl = 0
fdm.do_trim(1)

qbar_psf = fdm['aero/qbar-psf']
Sw_sqft = fdm['metrics/Sw-sqft']
cbar = fdm['metrics/cbarw-ft']
a0 = fdm['aero/alpha-rad']
b0 = fdm['aero/beta-rad']
p0 = fdm['velocities/p-aero-rad_sec']
q0 = fdm['velocities/q-aero-rad_sec']
r0 = fdm['velocities/r-aero-rad_sec']

aux = fdm.get_auxiliary()
body2stab = np.array(
    [
        [np.cos(a0),  0, np.sin(a0)],
        [0,          1, 0        ],
        [-np.sin(a0), 0, np.cos(a0)]
    ]
)
stab2wind = np.array(
    [
        [np.cos(b0),  np.sin(b0), 0],
        [-np.sin(b0), np.cos(b0), 0],
        [0,          0,         1]
    ]
)
assert np.all(aux.get_Tb2w() == stab2wind @ body2stab)

def get_coefficients() -> np.ndarray:
    fdm.run_ic()
    X = fdm['forces/fbx-aero-lbs']
    Y = fdm['forces/fby-aero-lbs']
    Z = fdm['forces/fbz-aero-lbs']
    F_bf = np.array([X, Y, Z])
    # coordinate transformation

    D, Y, L = np.array(aux.get_Tb2w()) @ F_bf
    CD = D / qbar_psf / Sw_sqft
    CY = Y / qbar_psf / Sw_sqft
    CL = L / qbar_psf / Sw_sqft
    l_bf = fdm['moments/l-aero-lbsft']
    m_bf = fdm['moments/m-aero-lbsft']
    n_bf = fdm['moments/n-aero-lbsft']
    M_bf = np.array([l_bf, m_bf, n_bf])
    l, m, n = np.array(aux.get_Tb2w()) @ M_bf
    Cl = l / qbar_psf / Sw_sqft / cbar
    Cm = m / qbar_psf / Sw_sqft / cbar
    Cn = n / qbar_psf / Sw_sqft / cbar
    return np.array([CD, CY, CL, Cl, Cm, Cn])

titles = ["CD", "CY", "CL", "Cl", "Cm", "Cn"]
trim_coeffs = zip(
    titles,
    get_coefficients()
)

def centered_diff_fourth_order(f, x, dx):
    num = -f(x + 2*dx) + 8 * f(x + dx) - 8 * f(x - dx) + f(x - 2*dx)
    return num/(12*dx)
# test difference finding code
assert np.isclose(centered_diff_fourth_order(lambda x: x**2, 1, 1e-4), 2)
th_test = np.linspace(0, 2*np.pi, 100)
assert np.allclose(centered_diff_fourth_order(lambda x: np.sin(x), th_test, 1e-1 * np.ones(100)), np.cos(th_test))

def coefficients_alpha(alpha_rad):
    fdm["ic/alpha-rad"] = alpha_rad
    return get_coefficients()

def coefficients_beta(beta):
    fdm["ic/beta-rad"] = beta
    return get_coefficients()

def coefficients_u(u):
    fdm["ic/vc-kts"] = u
    return get_coefficients()

def coefficients_p(p):
    fdm["ic/p-rad-sec"] = p
    return get_coefficients() 

def coefficients_q(q):
    fdm["ic/q-rad-sec"] = q
    return get_coefficients()

def coefficients_r(r):
    fdm["ic/r-rad-sec"] = r
    return get_coefficients()

da = np.radians(1e-3)
du = 1e-3
C_alpha = centered_diff_fourth_order(coefficients_alpha, a0, da)
C_beta = centered_diff_fourth_order(coefficients_beta, b0, da)
C_u = centered_diff_fourth_order(coefficients_u, u0, 1) * u0
C_p = centered_diff_fourth_order(coefficients_p, p0, da) * 2 * u0 / cbar
C_q = centered_diff_fourth_order(coefficients_p, q0, da) * 2 * u0 / cbar
C_r = centered_diff_fourth_order(coefficients_p, r0, da) * 2 * u0 / cbar

derivs = ["alpha", "beta", "u", "p", "q", "r"]
Cs = np.array([titles, C_alpha, C_beta, C_u, C_p, C_q, C_r]).T

print(tabulate(trim_coeffs))
print("="*10)
print(tabulate(Cs, headers = ["title"] + derivs, floatfmt=".3e"))