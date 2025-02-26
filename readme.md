# Even Flow Model

This repository contains the code necessary to run the custom model of Even Flow in flightgear / jsbsim.

To compile the aerodynamic model, you will need the `typer`, `pyparsing`, and `tabulate` python packages. Compile with

```
python3 aerodynamic-sexpr.py compile ./EvenFlow/EvenFlow_no-vi.sexpr > aircraft/EvenFlow/EvenFlowAerodynamics.xml
```

The 3D model of the airplane is currently very basic, but improved versions can be made in Blender.
