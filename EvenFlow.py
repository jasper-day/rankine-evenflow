# Define constants
from compile_python_to_jsbsim import Functions, compile_xml, print_xml, Wing_Panel, Fuselage, ROLL

functions = Functions()
# croot = 0.52
# ctip = 0.2
# root centroid = y: 0.75, x: -0.26 (M)
# tip centroid = y: 1.426, x: -0.19 (M)
# divide x-values by 2 to get the quarter chord location
# working to an AERORP at the CG to get the rotation arms correct
# CG located at 0.36 * 0.4287
cg_loc = -0.36 * 0.4287 # from LE
lw0 = Wing_Panel(name = "lw0", 
                unit = "M",
                x = -0.13 - cg_loc,
                y = -0.75,
                z = 0,
                u_z = 0, v_z = 0, w_z = 1,
                u_x = 1, v_x = 0, w_x = 0,
                a = 5.163,
                clmax = 1.1,
                k = 0.0464,
                cd0 = 0.0118,
                S = 0.52 * 0.75,
                f_name = None,
                tau_f = 0.0,
                propwash=None,
                downwash=None)
rw0 = Wing_Panel(name = "rw0",
                unit = "M",
                x = -0.13 - cg_loc,
                y = 0.75,
                z = 0,
                u_z = 0, v_z = 0, w_z = 1,
                u_x = 1, v_x = 0, w_x = 0,
                a = 5.163,
                clmax = 1.1,
                k = 0.0464,
                cd0 = 0.0118,
                S = 0.52 * 0.75,
                f_name = None,
                tau_f = 0.0,
                propwash = None,
                downwash = None)
lw1 = Wing_Panel(name = "lw1", 
                unit = "M",
                x = -0.19/2 - cg_loc,
                y = -1.426,
                z = 0,
                u_z = 0, v_z = 0, w_z = 1,
                u_x = 1, v_x = 0, w_x = 0,
                a = 5.163,
                clmax = 1.1,
                k = 0.0464,
                cd0 = 0.0118,
                S = 0.5 * (0.2 + 0.52),
                f_name = "left-aileron",
                tau_f = 0.7,
                propwash=None,
                downwash=None)
rw1 = Wing_Panel(name = "rw1",
                unit = "M",
                x = -0.13 - cg_loc,
                y = 1.426,
                z = 0,
                u_z = 0, v_z = 0, w_z = 1,
                u_x = 1, v_x = 0, w_x = 0,
                a = 5.163,
                clmax = 1.1,
                k = 0.0464,
                cd0 = 0.0118,
                S = 0.5 * (0.2 + 0.52),
                f_name = "right-aileron",
                tau_f = 0.7,
                propwash = None,
                downwash = None)

# ht at 3.1 mac from LE
ht = Wing_Panel(name = "ht",
                unit = "M",
                x = -3.1 * 0.4287 - cg_loc,
                y = 0.0,
                z = 0.0,
                u_z = 0, v_z = 0, w_z = 1,
                u_x = 1, v_x = 0, w_x = 0,
                a = 4.88,
                clmax = 1.0,
                k = 0.0513,
                cd0 = 0.0136,
                S = 0.26,
                f_name = "elevator",
                tau_f = 0.5,
                propwash = 0.0,
                downwash = 0.8)
# revised rudder dimensions
vt = Wing_Panel(name = "vt",
                unit = "M",
                x = -3.1 * 0.4287 - cg_loc,
                y = 0.0, 
                z = -102e-3,
                u_z = 0, v_z = 1, w_z = 0,
                u_x = 1, v_x = 0, w_x = 0,
                a = 4.88,
                clmax = 1.0,
                k = 0.0885,
                cd0 = 0.0136,
                S = 95068e-6,
                f_name = "rudder",
                tau_f = 0.7,
                propwash = 0.0,
                downwash = None)

fus = Fuselage(
    unit = "FT",
    x = 0.0, y = 0.0, z = 0.0,
    X_uu = 0.575, Y_vv = 2, Z_ww = 2,
    propwash = 0.0,
)

# induced velocity of the wing at the wing
# used for downwash calculations
# stevens 8.5-15
functions["aero/velocities/wing-zi-fps"] = f"""
(* (max 0 velocities/u-aero-fps)
   0.5
   (+ aero/coefficients/CL_lw0
      aero/coefficients/CL_rw0
      aero/coefficients/CL_lw1
      aero/coefficients/CL_rw1)
   0.0464)
"""

# Effect of sideslip on roll moment

# ROLL["aero/moments/L_beta-lbft"] = f"""

# """

if __name__ == "__main__":
    elements = [functions, 
                lw0, lw1, 
                rw0, rw1,
                ht, 
                vt, 
                fus,
                ]
    xml = compile_xml(elements)
    print(print_xml(xml))