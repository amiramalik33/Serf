##################################################
#   4 Degree of Freedom Simulation for 
#   Surface Splitting Hydrofoil Boat
#   roll, pitch, z, and x (p, q, z, x)
#   Forward Rising Sky-Pitching Right Roll is positive
#
#   Copyright (c) Amira Malik 2024
##################################################

vehicle_params = {
  "foilLength": 0,
  "foil_c": 0,
  "foil_incidence": 0,
  "foilDihedral": 0,
  "tailIncidence": 0,
  "tailArea": 0,
  "CGx": 0,
  "CGy": 0,
  "CGz": 0,
  "mass": 0,
  "thrust": 0,
  "pitchInertia": 0,
  "rollInertia": 0
}

env_params = {
    "air_rho": 1.225,
    "water_rho": 997
}