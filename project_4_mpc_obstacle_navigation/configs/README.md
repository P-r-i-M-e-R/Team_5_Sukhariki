Project configuration notes
===========================

The simulation parameters, robot parameters, map bounds, and validation
scenarios are defined in `src/environment.py`.

This small `configs/` folder is included to match the required project
layout. The project keeps parameters in Python dataclasses so the scenarios
can be validated and imported directly by the simulation code.
