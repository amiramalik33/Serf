##################################################
#   4 Degree of Freedom Simulation for 
#   Surface Splitting Hydrofoil Boat
#   roll, pitch, z, and x (p, q, z, x)
#   Forward Rising Sky-Pitching Right Roll is positive
#
#   Copyright (c) Amira Malik 2024
##################################################

from math import *
import fluids as flu
import compute_geometry as geo

####### ACTUAL COMPUTATION ##########

def compute_accels(veh_locs, veh_vels, vehicle_params, env_params):

    pitch = veh_locs[0]
    roll = veh_locs[1]
    x = veh_locs[2]
    z = veh_locs[3]

    pitchRate = veh_vels[0]
    rollRate = veh_vels[1]
    Vx = veh_vels[2]
    Vz = veh_vels[3]

    foilLength = vehicle_params["foilLength"]
    foilChord = vehicle_params["foilChord"]
    foilIncidence = vehicle_params["foilIncidence"]
    foilDihedral = vehicle_params["foilDihedral"]

    tailIncidence = vehicle_params["tailIncidence"]
    tailArea = vehicle_params["tailArea"]

    CGx = vehicle_params["CGx"]
    CGy = vehicle_params["CGy"]
    CGz = vehicle_params["CGz"]
    mass = vehicle_params["mass"]
    pitchInertia = vehicle_params["pitchInertia"]
    rollInertia = vehicle_params["rollInertia"]

    thrust = vehicle_params["thrust"]

    air_rho = env_params["air_rho"]
    water_rho = env_params["water_rho"]

    foil_aoa = foilIncidence + pitch
    tail_aoa = tailIncidence + pitch

    [R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater] = geo.submergence(pitch, roll, z, foilLength, foilDihedral)

    [R_rollarmAir, R_rollarmWater, L_rollarmAir, L_rollarmWater] = geo.roll_moment_arms(CGz, CGy, R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater)

    [R_pitcharmAir, R_pitcharmWater, L_pitcharmAir, L_pitcharmWater] = geo.pitch_moment_arms(CGx, CGz, R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater)

    R_L_water = flu.foil_lift(water_rho, R_lengthWater, foilChord, Vx, foil_aoa)
    R_L_air =   flu.foil_lift(air_rho, R_lengthAir, foilChord, Vx, foil_aoa)
    L_L_water = flu.foil_lift(water_rho, L_lengthWater, foilChord, Vx, foil_aoa)
    L_L_air =   flu.foil_lift(air_rho, L_lengthAir, foilChord, Vx, foil_aoa)

    R_D_water = flu.foil_drag(water_rho, R_lengthWater, foilChord, Vx, foil_aoa)
    R_D_air =   flu.foil_drag(air_rho, R_lengthAir, foilChord, Vx, foil_aoa)
    L_D_water = flu.foil_drag(water_rho, L_lengthWater, foilChord, Vx, foil_aoa)
    L_D_air =   flu.foil_drag(air_rho, L_lengthAir, foilChord, Vx, foil_aoa)

    tailLift = flu.tail_lift(Vx, tail_aoa, tailArea)
    tailDrag = flu.tail_drag(Vx, tail_aoa, tailArea)


    FZ = R_L_water + R_L_air + L_L_water + L_L_air - mass*9.81
    FX = thrust - (R_D_water + R_D_air + L_D_water + L_D_air)
    T_ROLL = L_L_water*L_rollarmWater + L_L_air*L_rollarmAir - (R_L_water*R_rollarmWater + R_L_air*R_rollarmAir)
    T_PITCH = (L_L_water*L_pitcharmWater + L_L_air*L_pitcharmAir + R_L_water*R_pitcharmWater + R_L_air*R_pitcharmAir) - (L_D_water*L_pitcharmWater + L_D_air*L_pitcharmAir + R_D_water*R_pitcharmWater + R_D_air*R_pitcharmAir + tailDrag + tailLift)

    Az = FZ/mass
    Ax = FX/mass
    rollAccel = T_ROLL/rollInertia
    pitchAccel = T_PITCH/pitchInertia

    return [pitchAccel, rollAccel, Ax, Az]