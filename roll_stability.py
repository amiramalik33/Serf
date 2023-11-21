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

def get_cL(aoa):
    """
    Returns lift coefficient given AOA
    """
    
    """
    #Simple Lookup Aero Database for Modified Airfoil
    
    cLs = {0.1:0.2,
           0.2:0.3,
           0.3:0.4
        }
    
    try:
        cL = cLs[round(aoa,1)]
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def get_lift(b, c, v, aoa):
    """
    Get lift for a rectangular wing
    Uses aero database to look up cL given AOA
    """
    
    rho = 998
    
    cL = get_cL(aoa)
    
    L = 1/2*cL*b*c*rho*(v**2)
    
    return L

def get_FoilForces(phi, gamma, Ll, Rl, lb, h, c, v, aoa):
    """
    Get torques and forces of foils
    Calls get_lengths and get_lift for each foil surface
    """
    
    Ls = get_lengths(phi, gamma, Ll, Rl, lb, h)
    
    Laa = Ls[0]
    Lwa = Ls[1]
    Raa = Ls[2]
    Rwa = Ls[3]
    Lwl = Ls[4]
    Lal = Ls[5]
    Rwl = Ls[6]
    Ral = Ls[7]
    
    F_LW = get_lift(Lwl, c, v, aoa)
    F_LA = get_lift(Lal, c, v, aoa)
    F_RW = get_lift(Rwl, c, v, aoa)
    F_RA = get_lift(Ral, c, v, aoa)
    
    T_LW = F_LW*Lwa
    T_LA = F_LA*Laa
    T_RW = F_RW*Rwa
    T_RA = F_RA*Raa
    
    F = F_LW + F_LA + F_RW + F_RA
    
    T = (T_LW + T_LA) - (T_RW + T_RA)
    
    return [F, T]
    
def compute_accels(I, m, phi, gamma, Ll, Rl, lb, h, c, v, aoa):
    
    FT = get_FoilForces(phi, gamma, Ll, Rl, lb, h, c, v, aoa)
    
    F = FT[0]
    T = FT[1]
    
    d2phi_dt2 = T/I
    
    d2y_dt2 = F/m
    
    return [d2phi_dt2, d2y_dt2]
    
    
def FE_next(U, dt, I, m, phi, gamma, Ll, Rl, lb, h, c, v, aoa):
    
    accels = compute_accels(I, m, phi, gamma, Ll, Rl, lb, h, c, v, aoa)

    #U = [t, x, dxdt, y, dydt, z, dzdt, p, dpdt, q, dqdt, r, drdt]
    """
    t, time
    x, forward position
    dxdt is related to z at steady state
    y, lateral position (maybe not needed)
    z, height above water
    p, roll
    q, pitch
    r, yaw
    """
    
    #THE REST OF THIS IS COPY PASTE FROM TAKEOFF SIM
    t      = U(1, -1);
    y      = U(2, -1);
    phi    = U(3, -1);
    dydt   = U(4, -1);
    dphidt = U(5, -1);

    d2phi_dt2 = accels[0]
    d2y_dt2 = accels[1] - m*9.81
    
    #Forward Euler
    t_next = t + dt
    
    dydt_next = dydt + d2y_dt2*dt
    y_next = y + dydt*dt
    
    dphidt_next = dphidt + d2phi_dt2*dt
    phi_next = phi + dphidt*dt
    
    U_next = [t_next, y_next, phi_next, dydt_next, dphidt_next]
    
    return U_next

    
    
    