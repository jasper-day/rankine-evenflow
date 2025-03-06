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

# Global variables that must be modified to match your particular need
# The aircraft name
# Note - It should match the exact spelling of the model file
AIRCRAFT_NAME = "YardStik"
# Path to JSBSim files, location of the folders "aircraft", "engines" and "systems"
PATH_TO_JSBSIM_FILES = "."

# Avoid flooding the console with log messages
jsbsim.FGJSBBase().debug_lvl = 0

fdm = jsbsim.FGFDMExec(PATH_TO_JSBSIM_FILES)
pm = fdm.get_property_manager()

# Load the aircraft model
fdm.load_model(AIRCRAFT_NAME)

# Set alpha range for trim solutions
fdm["aero/alpha-max-rad"] = math.radians(15)
fdm["aero/alpha-min-rad"] = math.radians(-5.0)

fdm.set_dt(1e-2)
dt = fdm.get_delta_t()
print("dt:", dt)

# Max control deflection
aileronMax = 0.0
rudderMax = 0.0

# Number of seconds for control surface to reach max deflection
risetime = 4

# Per timestep increment for control surfaces
diAileron = aileronMax / (risetime / dt)
diRudder = rudderMax / (risetime / dt)


# Initial conditions
fdm["ic/h-sl-ft"] = 1000
fdm["ic/vc-kts"] = 20
fdm["ic/gamma-deg"] = 0
fdm["ic/beta-deg"] = 0

# Initialize the aircraft with initial conditions
fdm["propulsion/engine[0]/set-running"] = 1
fdm["propulsion/engine[1]/set-running"] = 1
fdm.run_ic()

# # Set engines running

# print("Number of engines", propulsion.get_num_engines())
# propulsion = fdm.get_propulsion()
# propulsion.get_engine(0).init_running()  # start the engine
# propulsion.get_steady_state()  # set it to the correct value


print("throttle", fdm["fcs/throttle-cmd-norm"])

jsbsim.FGJSBBase().debug_lvl = 1
# Trim
try:
    fdm.do_trim(1)

except jsbsim.TrimFailureError:
    print("Trim failed, continuing rudder kick in an untrimmed state.")

    pass  # Ignore trim failure

linearization = jsbsim.FGLinearization(fdm)
A = linearization.system_matrix
evals, evecs = np.linalg.eig(A)
print("eigenvalues", evals)
# print("eigenvectors", evecs)
# plt.scatter(np.real(evals), np.imag(evals))
# plt.axhline(0)
# plt.axvline(0)
# plt.grid()
# plt.show()
plt.spy(A, precision=1e-3)
plt.show()

# # Time to run for in seconds
# run_period = 20

# # fdm.enable_output()
# # fdm.hold()
# # while True:
# #     fdm.run()
# #     fdm.hold()
# #     time.sleep(0.1)
# # Recorded data
# results = dict(
#     times=[],
#     betas=[],
#     bankAngle=[],
#     ailerons=[],
#     rudder=[],
#     alphas=[],
#     hsl=[],
#     thrust=[],
#     x=[],
#     y=[],
#     vc=[],
# )

# for i in range(int(run_period / dt)):
#     fdm.run()

#     results["times"].append(fdm.get_sim_time())

#     results["betas"].append(fdm["aero/beta-deg"])
#     results["bankAngle"].append(fdm["attitude/phi-deg"])
#     results["ailerons"].append(fdm["fcs/aileron-cmd-norm"])
#     results["rudder"].append(fdm["fcs/rudder-cmd-norm"])
#     results["alphas"].append(fdm["aero/alpha-deg"])
#     results["hsl"].append(fdm["position/h-sl-ft"])
#     results["thrust"].append(fdm["propulsion/engine[0]/thrust-lbs"])
#     results["x"].append(fdm["position/distance-from-start-lat-mt"])
#     results["y"].append(fdm["position/distance-from-start-lon-mt"])
#     results["vc"].append(fdm["velocities/vc-kts"])

#     aileronCmd = fdm["fcs/aileron-cmd-norm"]
#     rudderCmd = fdm["fcs/rudder-cmd-norm"]

#     if aileronCmd < aileronMax:
#         aileronCmd += diAileron
#         fdm["fcs/aileron-cmd-norm"] = aileronCmd

#     if rudderCmd < rudderMax:
#         rudderCmd += diRudder
#         fdm["fcs/rudder-cmd-norm"] = rudderCmd

# # Plot results
# df = pd.DataFrame(results)

# ax1 = plt.subplot(211)
# ax1.set_xlabel("Time (s)")
# ax1.set_ylabel("deg")
# sns.set_style(style="whitegrid")
# sns.lineplot(df, x="times", y="betas", label="Beta", color="red", ax=ax1)
# sns.lineplot(df, x="times", y="alphas", ax=ax1, label="alphas")
# plt.legend(loc="lower left")

# ax2 = ax1.twinx()

# sns.lineplot(df, x="times", y="hsl", ax=ax2, label="height", color="C4")

# ax3 = plt.subplot(212)
# sns.lineplot(x=df.times[2:], y=df.thrust[2:], ax=ax3)

# ax4 = ax3.twinx()
# sns.lineplot(df, x="times", y="vc", color="C1")

# plt.title("Rudder Kick")

# plt.tight_layout()
# plt.show()

# # ax3d = plt.figure().add_subplot(projection="3d")

# # ax3d.plot(df.x, df.y, df.hsl)
# # ax3d.set_aspect("equal")
# # ax3d.set_xlabel("X")
# # ax3d.set_ylabel("Y")
# # ax3d.set_zlabel("Z")

# # plt.show()
