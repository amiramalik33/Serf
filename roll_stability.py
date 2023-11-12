"""
Roll Stability of a Surface-Piercing Hydrofoil Boat with a single "V-Foil"

Given a symmetric V-foil, length of foil, angle of foil to vertical axis, and
the length of the foil to the CG of the boat, a system response can be made.

Hopefully, I will also be able to inject disturbances to height and angle

TO MATH OUT:
    Find max bank angle as function of foil length
    Instead of vehicle database, just run simple lift equation: VDB can come later!
    Forces to Torques (including angles :( )
    is left bank angle positive or negative? idk, but make sure you're consistent
    transform the CG downward (the cg is currently assumed to be at the 
                               intersection of the two foils extended above
                               the boat, which is waaaay too high)
    somehow make sure I can inject noise into h (height) and phi (bank angle)
"""

from math import *

def get_lengths(phi, gamma, Ll, Rl, lb, h):
    """
    Based on some geometry, we can get:
    Laa - the moment arm to the midpoint of the left foil above water
    Lwa - the moment arm to the midpoint of the left foil below water
    Raa - the moment arm to the midpoint of the right foil above water
    Rwa - the moment arm to the midpoint of the right foil below water
    Lwl - the length of submerged left foil
    Lal - the length of unsubmerged left foil
    Rwl - the length of submerged right foil
    Ral - the length of unsubmerged right foil
    
    given the:
    
    phi - bank angle, in radians, left = positive
    gamma - the foil angle to vertical axis, in radians
    Ll - total length of left foil
    Rl - total length of right foil (= Ll)
    lb - the length of the foil inside the boat to the cg
    """
    
    #Get foil length in air
    if phi >= 0:
        Lal = h/sin(pi/2-phi-gamma) - lb
        Ral = h/cos(phi+gamma) - lb
    elif phi < 0:
        phi = abs(phi)
        Ral = h/sin(pi/2-phi-gamma) - lb
        Lal = h/cos(phi+gamma) - lb   
    else:
        raise Exception('Phi undefined in get_lengths')
    
    #Get foil length in water
    Lwl = Ll - Lal - lb
    Rwl = Rl - Ral - lb
    
    #Get moment arms for air and water forces
    Laa = lb + Lal/2
    Lwa = lb + Lal + Lwl/2
    Raa = lb + Ral/2
    Rwa = lb + Ral + Rwl/2
    
    return [Laa, Lwa, Raa, Rwa, Lwl, Lal, Rwl, Ral]