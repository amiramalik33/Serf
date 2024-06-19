##################################################
#   4 Degree of Freedom Simulation for 
#   Surface Splitting Hydrofoil Boat
#   roll, pitch, z, and x (p, q, z, x)
#
#   Copyright (c) Amira Malik 2024
##################################################

def submergence(pitch, roll, z, foilLength, foilDihedral):
    """
    Returns distances of foils in air and water as function of roll (p) & pitch (q) angle
    """

    R_lengthAir = 0
    R_lengthWater = 0

    L_lengthAir = 0
    L_lengthWater = 0
    
    return [R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater]

def roll_moment_arms(CGz, CGy, R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater):
    """
    Returns 2D moment arms in front-facing view for roll acceleration
    """

    R_rollarmAir = 0
    R_rollarmWater = 0

    L_rollarmAir = 0
    L_rollarmWater = 0


    return [R_rollarmAir, R_rollarmWater, L_rollarmAir, L_rollarmWater]

def pitch_moment_arms(CGx, CGz, R_lengthAir, R_lengthWater, L_lengthAir, L_lengthWater):
    """
    Returns 2D moment arms in side-facing view for pitch acceleration
    """

    R_pitcharmAir = 0
    R_pitcharmWater = 0

    L_pitcharmAir = 0
    L_pitcharmWater = 0

    return [R_pitcharmAir, R_pitcharmWater, L_pitcharmAir, L_pitcharmWater]