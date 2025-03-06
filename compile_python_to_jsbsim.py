# Python library for compiling python documents into JSBSim FDM
# The goal here is to make defining wings a little bit less painful

from dataclasses import dataclass
from xml.etree import ElementTree as ET
from typing import List, Literal
from compile_sexpr import spec, sexp
import numpy as np
import pint; u = pint.UnitRegistry()

m2ft = (1*u.m).to(u.ft).magnitude

class FDM_Element:
    def __init__(self):
        self._dictionary = {}
    def __setitem__(self, key: str, value: str | float):
        self._dictionary[key] = value
    def __getitem__(self, key: str):
        return self._dictionary[key]
    def xml_item(self, key, value):
        raise NotImplementedError
    def add_to(self, root):
        for key, value in self._dictionary.items():
            xml = self.xml_item(key, value)
            if not isinstance(xml, ET.Element):
                raise Exception(f"{key}: {value} could not be converted to xml")
            root.append(xml)
    
class Constants(FDM_Element):
    def xml_item(self, key, value):
        fn_element = ET.Element("function")
        fn_element.set("name", key)
        value_element = ET.Element("value")
        value_element.text = f" {value} "
        fn_element.append(value_element)
        return fn_element

class Functions(FDM_Element):
    def xml_item(self, key, value):
        fn_element = ET.Element("function")
        fn_element.set("name", key)
        try:
            children = sexp.parse_string(value)
        except:
            raise Exception("could not parse", value)
        for element in children:
            if isinstance(element, ET.Element):
                fn_element.append(element)
        return fn_element

class Axis(Functions):
    def __init__(self, name: Literal["X", "Y", "Z", "ROLL", "PITCH", "YAW"]):
        super().__init__()
        self.name = name
    def add_to(self, root):
        axis_element = ET.Element("axis")
        axis_element.set("name", self.name)
        axis_element.set("frame", "BODY")
        super().add_to(axis_element)
        root.append(axis_element)

# define the axes directly in the library
X = Axis("X")
Y = Axis("Y")
Z = Axis("Z")
ROLL = Axis("ROLL")
PITCH = Axis("PITCH")
YAW = Axis("YAW")

class Wing_Panel(FDM_Element):
    "A generic wing panel. Must be added after the axis elements."
    def __init__(self, 
                 name: str,
                 unit: Literal["FT", "M"],
                 x: float, y: float, z: float,
                 u_z: float, v_z: float, w_z: float,
                 u_x: float, v_x: float, w_x: float,
                 a: float, clmax: float, k: float, cd0: float,
                 S: float,
                 f_name: Literal["right-aileron", "left-aileron", "rudder", "elevator"] | None, 
                 tau_f: float | None,
                 propwash: float | None, 
                 downwash: float | None
                 ):
        super().__init__()
        w = name
        if unit == "M":
            x = x * m2ft
            y = y * m2ft
            z = z * m2ft
            S = S * m2ft**2
        # Normalize normal vectors
        len_x_norm = (u_x**2 + w_x**2 + v_x**2)**0.5
        u_x = u_x / len_x_norm; v_x = v_x / len_x_norm; w_x = w_x / len_x_norm
        len_z_norm = (u_z**2 + w_z**2 + v_z**2)**0.5
        u_z = u_z / len_z_norm; v_z = v_z / len_z_norm; w_z = w_z / len_z_norm
        alphamax = clmax / a
        constants = Constants()
        functions = Functions()
        # body positions
        constants[f"aero/quantity/x_{w}-ft"] = x
        constants[f"aero/quantity/y_{w}-ft"] = y
        constants[f"aero/quantity/z_{w}-ft"] = z
        constants[f"aero/coefficients/CD0_{w}"] = cd0
        constants[f"aero/coefficients/k_{w}"] = k
        constants[f"aero/metrics/S_{w}-sqft"] = S
        if propwash is not None:
            functions[f"aero/velocities/prop-{w}-ui-fps"] = \
                f"(* {propwash} propulsion/engine/prop-induced-velocity_fps)"
        else:
            functions[f"aero/velocities/prop-{w}-ui-fps"] = "0.0"
        if downwash is not None:
            functions[f"aero/velocities/wing-{w}-zi-fps"] = \
                f"(* -1.0 aero/velocities/wing-zi-fps {downwash})"
        else:
            functions[f"aero/velocities/wing-{w}-zi-fps"] = "0.0"
        if f_name is not None:
            functions[f"aero/calculated/delta-alpha_{w}_{f_name}-rad"] = \
                f"(* {tau_f} fcs/{f_name}-pos-rad)"
        else:
            functions[f"aero/calculated/delta-alpha_{w}_{f_name}-rad"] = "0.0"
        # local velocity vector, body frame
        # v = U + omega x r
        # account for propwash and downwash
        functions[f"aero/velocities/U_{w}_bf-fps"] = f"""
(+ velocities/u-aero-fps
   (* velocities/q-aero-rad_sec
      aero/quantity/z_{w}-ft)
   (* -1.0
      velocities/r-aero-rad_sec
      aero/quantity/y_{w}-ft)
   aero/velocities/prop-{w}-ui-fps)
"""
        functions[f"aero/velocities/V_{w}_bf-fps"] = f"""
(+ velocities/v-aero-fps
   (* velocities/r-aero-rad_sec
      aero/quantity/x_{w}-ft)
   (* -1.0
      velocities/p-aero-rad_sec
      aero/quantity/z_{w}-ft))
"""
        functions[f"aero/velocities/W_{w}_bf-fps"] = f"""
(+ velocities/w-aero-fps
   (* velocities/p-aero-rad_sec
      aero/quantity/y_{w}-ft)
   (* -1.0
      velocities/q-aero-rad_sec
      aero/quantity/x_{w}-ft)
   aero/velocities/wing-{w}-zi-fps)
"""
        # spanwise velocity U (wing frame)
        functions[f"aero/velocities/U_{w}_wf-fps"] = f"""
(+ (* {u_x} aero/velocities/U_{w}_bf-fps)
   (* {v_x} aero/velocities/V_{w}_bf-fps)
   (* {w_x} aero/velocities/W_{w}_bf-fps))
        """
        # normal velocity W (wing frame)
        functions[f"aero/velocities/W_{w}_wf-fps"] = f"""
(+ (* {u_z} aero/velocities/U_{w}_bf-fps)
   (* {v_z} aero/velocities/V_{w}_bf-fps)
   (* {w_z} aero/velocities/W_{w}_bf-fps))
"""
        # effective dynamic
        functions[f"aero/calculated/qbar_{w}-psf"] = f"""
(* 0.5
   atmosphere/rho-slugs_ft3
   (+ (pow aero/velocities/U_{w}_wf-fps 2)
      (pow aero/velocities/W_{w}_wf-fps 2)))
"""
        # angle of attack
        functions[f"aero/calculated/alpha_{w}-rad"] = f"""
(+ (atan2 aero/velocities/W_{w}_wf-fps
          aero/velocities/U_{w}_wf-fps)
   aero/calculated/delta-alpha_{w}_{f_name}-rad)
"""

        # lift coefficient
        functions[f"aero/coefficients/CL_{w}"] = f"""
(table aero/table/CL_{w}_alpha
  (row aero/calculated/alpha_{w}-rad)
  [-1.57 0,
  {-alphamax} {-clmax},
  {alphamax} {clmax},
  1.57 0])"""
        # separation drag
        functions[f"aero/coefficients/CD-sep_{w}"] = f"""
(table aero/table/CD-sep_{w}_alpha
    (row aero/calculated/alpha_{w}-rad)
    [-1.57 1,
    {-alphamax} 0,
    {alphamax} 0,
    1.57 1])"""
        # drag coefficient
        functions[f"aero/coefficients/CD_{w}"] = f"""
(+ aero/coefficients/CD0_{w}
   (* aero/coefficients/k_{w}
      (pow aero/coefficients/CL_{w} 2))
   aero/coefficients/CD-sep_{w})
"""
        # lift force
        functions[f"aero/forces/L_{w}-lb"] = f"""
(* aero/coefficients/CL_{w}
   aero/metrics/S_{w}-sqft
   aero/calculated/qbar_{w}-psf)
"""
        # drag force
        functions[f"aero/forces/D_{w}-lb"] = f"""
(* aero/coefficients/CD_{w}
   aero/metrics/S_{w}-sqft
   aero/calculated/qbar_{w}-psf)
"""
        # wing frame forces
        functions[f"aero/forces/X_{w}_wf-lb"] = f"""
(+ (* aero/forces/L_{w}-lb
      (sin aero/calculated/alpha_{w}-rad))
   (* -1.0 aero/forces/D_{w}-lb
      (cos aero/calculated/alpha_{w}-rad)))
"""
        functions[f"aero/forces/Z_{w}_wf-lb"] = f"""
(+ (* -1.0 aero/forces/L_{w}-lb
      (cos aero/calculated/alpha_{w}-rad))
   (* -1.0 aero/forces/D_{w}-lb
      (sin aero/calculated/alpha_{w}-rad)))
"""
        
        X[f"aero/forces/X_{w}-lb"] = f"""
(+ (* {u_x} aero/forces/X_{w}_wf-lb)
   (* {u_z} aero/forces/Z_{w}_wf-lb))
"""
        Y[f"aero/forces/Y_{w}-lb"] = f"""
(+ (* {v_x} aero/forces/X_{w}_wf-lb)
   (* {v_z} aero/forces/Z_{w}_wf-lb))
"""
        Z[f"aero/forces/Z_{w}-lb"] = f"""
(+ (* {w_x} aero/forces/X_{w}_wf-lb)
   (* {w_z} aero/forces/Z_{w}_wf-lb))
"""
        ROLL[f"aero/moments/L_{w}-ftlb"] = f"""
(+ (* -1.0 aero/quantity/z_{w}-ft
      aero/forces/Y_{w}-lb)
   (* aero/quantity/y_{w}-ft
      aero/forces/Z_{w}-lb))
"""
        PITCH[f"aero/moments/M_{w}-ftlb"] = f"""
(+ (* -1.0 aero/quantity/x_{w}-ft
      aero/forces/Z_{w}-lb)
   (* aero/quantity/z_{w}-ft
      aero/forces/X_{w}-lb))
"""
        YAW[f"aero/moments/N_{w}-ftlb"] = f"""
(+ (* -1.0 aero/quantity/y_{w}-ft
      aero/forces/X_{w}-lb)
   (* aero/quantity/x_{w}-ft
      aero/forces/Y_{w}-lb))
"""
        self.constants = constants
        self.functions = functions

    def add_to(self, root):
        # add generic functions first
        # then add to axes
        self.constants.add_to(root)
        self.functions.add_to(root)

class Fuselage(Functions):
    def __init__(self, unit: Literal["FT", "M"],
                 x: float, y: float, z: float, 
                 X_uu: float, Y_vv: float, Z_ww: float,
                 propwash: float = 0.0):
        super().__init__()
        self["aero/velocities/fus-u-fps"] = f"""
(+ velocities/u-aero-fps
   (* {propwash} propulsion/engine/prop-induced-velocity_fps))
"""
        X["aero/forces/X_fus-lb"] = f"""
(* -0.5
   atmosphere/rho-slugs_ft3
   {X_uu}
   (abs aero/velocities/fus-u-fps)
   aero/velocities/fus-u-fps)
"""
        Y["aero/forces/Y_fus-lb"] = f"""
(* -0.5
   atmosphere/rho-slugs_ft3
   {Y_vv}
   (abs velocities/v-aero-fps)
   velocities/v-aero-fps)
"""
        Z["aero/forces/Z_fus-lb"] = f"""
(* -0.5
   atmosphere/rho-slugs_ft3
   {Z_ww}
   (abs velocities/w-aero-fps)
   velocities/w-aero-fps)
"""
        ROLL["aero/moments/L_fus-ftlb"] = f"""
(+ (* -1.0 {z}
      aero/forces/Y_fus-lb)
   (* {y}
      aero/forces/Z_fus-lb))
"""
        PITCH["aero/moments/M_fus-ftlb"] = f"""
(+ (* -1.0 {x}
      aero/forces/Z_fus-lb)
   (* {z}
      aero/forces/X_fus-lb))
"""
        YAW["aero/moments/N_fus-ftlb"] = f"""
(+ (* -1.0 {y}
      aero/forces/X_fus-lb)
   (* {x}
      aero/forces/Y_fus-lb))
"""
        

def compile_xml(elements):
    root = ET.Element("aerodynamics")
    for element in elements:
        element.add_to(root)
    for axis in [X, Y, Z, ROLL, PITCH, YAW]:
        axis.add_to(root)
    return root

def print_xml(root) -> str:
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")