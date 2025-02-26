# Simple code to display the lift curve slope of the plane

import jsbsim
import numpy as np
import matplotlib.pyplot as plt

# Load the aircraft model
AIRCRAFT_NAME = "EvenFlow"
PATH_TO_JSBSIM_FILES = "."
# Avoid flooding the console with log messages
jsbsim.FGJSBBase().debug_lvl = 0
fdm = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)
fdm.load_model(AIRCRAFT_NAME)

aux = fdm.get_auxiliary()

# trim condition
# steady level flight

# Initial conditions

def get_CL(alpha):
    u0 = 30
    fdm["ic/h-sl-ft"] = 1000
    fdm["ic/vc-kts"] = u0
    fdm["ic/beta-deg"] = 0
    fdm["ic/alpha-rad"] = alpha

    # Initialize the aircraft with initial conditions
    fdm["propulsion/engine/set-running"] = 1
    fdm.run_ic()

        # try:
        #     fdm.do_trim(4)
        # except jsbsim.TrimFailureError as e:
        #     print("failed to trim", alpha)
        #     pass

    qbar_area = fdm['aero/qbar-area'] # equal to qbar-psf * Sw_sqft
    # I'm so lost with the coordinate definitions
    X = -fdm['forces/fbx-aero-lbs']
    Y = fdm['forces/fby-aero-lbs']
    Z = -fdm['forces/fbz-aero-lbs']

    D, C, L = np.array(aux.get_Tb2w()) @ np.array([X, Y, Z])
    assert np.sign(D) == 1.0, f"Drag = {D} at alpha = {alpha}"
    return [L / qbar_area, D / qbar_area]

alpha = np.radians(np.linspace(-15, 15, 20))
Cs = np.array([get_CL(a) for a in alpha])
CL = Cs[:, 0]
CD = Cs[:, 1]
a_w = 5.163
plt.subplot(121)
plt.plot(np.degrees(alpha), CL, label="Simulated")
plt.plot(np.degrees(alpha), a_w * alpha, label="Main wing")
plt.xlabel("alpha (deg)")
plt.ylabel("CL")
plt.grid()
plt.legend()
plt.subplot(122)
plt.plot(np.degrees(alpha), CD, label="Simulated")
plt.plot(np.degrees(alpha), 0.04 * (a_w * alpha)**2 + 0.061, label="Theoretical")
plt.grid()
plt.legend()
plt.show()