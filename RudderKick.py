# RudderKick.py
#
# Simulate a pilot performing a rudder kick test by inputing a rudder input based
# on a ramp input. Aileron input is also included to maintain a steady heading
# sideslip (SHSS). The time histories of the control inputs and beta (sideslip angle)
# are plotted.
#
# An equivalent JSBSim XML script version can be found in scripts\rudder_kick.xml
#
# Copyright (c) 2023 Sean McLeod
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>
#

import jsbsim
import matplotlib.pyplot as plt
import math
import pandas as pd
import seaborn as sns
import numpy as np
from tabulate import tabulate

# Global variables that must be modified to match your particular need
# The aircraft name
# Note - It should match the exact spelling of the model file
AIRCRAFT_NAME = "EvenFlow"
# Path to JSBSim files, location of the folders "aircraft", "engines" and "systems"
PATH_TO_JSBSIM_FILES = "."

# Avoid flooding the console with log messages
jsbsim.FGJSBBase().debug_lvl = 0

fdm = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)

# Load the aircraft model
fdm.load_model(AIRCRAFT_NAME)

# Set alpha range for trim solutions
fdm["aero/alpha-max-rad"] = math.radians(15)
fdm["aero/alpha-min-rad"] = math.radians(-5.0)

fdm.set_dt(1e-3)
dt = fdm.get_delta_t()

# Max control deflection
aileronMax = 0.3
rudderMax = 0.2

# Number of seconds for control surface to reach max deflection
risetime = 4

# Per timestep increment for control surfaces
diAileron = aileronMax / (risetime / dt)
diRudder = rudderMax / (risetime / dt)


# Initial conditions
fdm["ic/h-sl-ft"] = 1000
fdm["ic/vc-kts"] = 30
fdm["ic/gamma-deg"] = 0
fdm["ic/beta-deg"] = 0

# Initialize the aircraft with initial conditions
fdm["propulsion/engine/set-running"] = 1
fdm.run_ic()

# # Set engines running

# print("Number of engines", propulsion.get_num_engines())
# propulsion = fdm.get_propulsion()
# propulsion.get_engine(0).init_running()  # start the engine
# propulsion.get_steady_state()  # set it to the correct value
# for alpha in range(0,15,3):
#     print("alpha", alpha)
#     fdm["aero/alpha-deg"] = alpha
#     fdm.run()
#     print("Aerodynamic forces")
#     forces = [
#         ["x", fdm["forces/fbx-aero-lbs"]],
#         ["y", fdm["forces/fby-aero-lbs"]],
#         ["z", fdm["forces/fbz-aero-lbs"]]
#         ]
#     print(tabulate(forces, tablefmt="plain"))
#     print("Accelerations")
#     accelerations = [
#         ["x", fdm["accelerations/udot-ft_sec2"]],
#         ["y", fdm["accelerations/vdot-ft_sec2"]],
#         ["z", fdm["accelerations/wdot-ft_sec2"]],
#         ]
#     print(tabulate(accelerations, tablefmt="plain"))
#     print("-"*10)

jsbsim.FGJSBBase().debug_lvl = 1
# Trim
try:
    fdm.do_trim(1)

except jsbsim.TrimFailureError:
    print("Trim failed, continuing rudder kick in an untrimmed state.")

    pass  # Ignore trim failure

# linearization = jsbsim.FGLinearization(fdm)
# A = linearization.system_matrix
# evals, evecs = np.linalg.eig(A)
# print("eigenvalues", evals)
# # print("eigenvectors", evecs)
# plt.scatter(np.real(evals), np.imag(evals))
# plt.axhline(0)
# plt.axvline(0)
# plt.grid()
# plt.show()
# plt.matshow(A)
# plt.show()

# # Time to run for in seconds
run_period = 20

# # fdm.enable_output()
# # fdm.hold()
# # while True:
# #     fdm.run()
# #     fdm.hold()
# #     time.sleep(0.1)
# Recorded data
results = []

for i in range(int(run_period / dt)):
    fdm.run()

    results.append([fdm.get_sim_time(),
        fdm["aero/beta-deg"],
        fdm["attitude/phi-deg"],
        fdm["fcs/aileron-cmd-norm"],
        fdm["fcs/rudder-cmd-norm"],
        fdm["aero/alpha-deg"],
        fdm["position/h-sl-ft"],
        fdm["propulsion/engine[0]/thrust-lbs"],
        fdm["position/distance-from-start-lat-mt"],
        fdm["position/distance-from-start-lon-mt"],
        fdm["velocities/vc-kts"],
        fdm["aero/coefficients/CL_lw"] + fdm["aero/coefficients/CL_rw"],
        fdm["aero/coefficients/CL_ht"],
        fdm["aero/coefficients/CL_vt"],
        fdm["forces/fbx-aero-lbs"],
        fdm["forces/fby-aero-lbs"],
        fdm["forces/fbz-aero-lbs"],
    ])

    aileronCmd = fdm["fcs/aileron-cmd-norm"]
    rudderCmd = fdm["fcs/rudder-cmd-norm"]

    if aileronCmd < aileronMax:
        aileronCmd += diAileron
        fdm["fcs/aileron-cmd-norm"] = aileronCmd

    if rudderCmd < rudderMax:
        rudderCmd += diRudder
        fdm["fcs/rudder-cmd-norm"] = rudderCmd
    
    if fdm["position/h-sl-ft"] < 10:
        break

# Plot results
df = pd.DataFrame(results, columns=[
    "times",
    "betas", "phi", "aileron", "rudder", "alphas", 
    "height", "thrust",
    "x", "y", "vc", "CL_w", "CL_ht", "CL_vt",
    "fx", "fy", "fz"
])


sns.set_style(style="whitegrid")
plt.subplot(321)
sns.lineplot(df, x="times", y="betas", label="beta")
sns.lineplot(df, x="times", y="alphas", label="alpha")
plt.subplot(322)
sns.lineplot(df, x="times", y="height")
plt.subplot(323)
sns.lineplot(df, x="times", y="rudder", label="rudder")
sns.lineplot(df, x="times", y="aileron", label="aileron")
plt.subplot(324)
sns.lineplot(df, x="times", y="vc")
plt.subplot(325)
sns.lineplot(df, x="times", y="CL_w")
plt.subplot(326)
sns.lineplot(df, x="times", y="CL_ht", label="H tail")
sns.lineplot(df, x="times", y="CL_vt", label="V tail")


plt.title("Rudder Kick")

plt.tight_layout()
plt.show()

# ax3d = plt.figure().add_subplot(projection="3d")

# ax3d.plot(df.x, df.y, df.hsl)
# ax3d.set_aspect("equal")
# ax3d.set_xlabel("X")
# ax3d.set_ylabel("Y")
# ax3d.set_zlabel("Z")

# plt.show()
