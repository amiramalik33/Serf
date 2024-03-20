##################################################
#   Library that contains all functions related to fluid dynamics
#
#   Author: Amira Malik
#
#   Copyright (c) Amira Malik
##################################################

##################################################
#   TO DOs:
#   Is it faster to load .csv files in main script and pass the list vs calling the csv file each time?
#   
#   
##################################################

def foil_cL(aoa):
    """
    Returns lift coefficient given AOA
    """
    
    """
    import csv "database" of aoa & cd

    linear interpolation between points
        if aoa = 1.345, get aoa = 1.3 and aoa = 1.4, then approx cd linearly
    
    try:
        (interpolation)
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def foil_cD(aoa):
    """
    Returns drag coefficient given AOA
    """
    
    """
    import csv "database" of aoa & cd

    linear interpolation between points
        if aoa = 1.345, get aoa = 1.3 and aoa = 1.4, then approx cd linearly
    
    try:
        (interpolation)
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def tail_cL(aoa):
    """
    Returns drag coefficient given AOA
    """
    
    """
    import csv "database" of aoa & cd

    linear interpolation between points
        if aoa = 1.345, get aoa = 1.3 and aoa = 1.4, then approx cd linearly
    
    try:
        (interpolation)
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def tail_cD(aoa):
    """
    Returns drag coefficient given AOA
    """
    
    """
    import csv "database" of aoa & cd

    linear interpolation between points
        if aoa = 1.345, get aoa = 1.3 and aoa = 1.4, then approx cd linearly
    
    try:
        (interpolation)
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def foil_lift(rho, b, c, v, aoa):
    """
    Get lift for a rectangular foil
    """
    
    cL = foil_cL(aoa)
    
    L = 1/2*cL*b*c*rho*(v**2)
    
    return L

def foil_drag(rho, b, c, v, aoa):
    """
    Get lift for a rectangular foil
    """
    
    cD = foil_cD(aoa)
    
    D = 1/2*cD*b*c*rho*(v**2)
    
    return D

def tail_lift(v, aoa, S):
    """
    Get lift for submerged tail
    """
    rho = 998

    cL = tail_cL(aoa)
    
    L = 1/2*cL*S*rho*(v**2)
    
    return L

def tail_drag(v, aoa, S):
    """
    Get drag for submerged tail
    """

    rho = 998

    cD = tail_cD(aoa)

    D = 1/2*cD*S*rho*(v**2)

    return D