import numpy as np


def method1(vxs, vys):
    fl = 5 # fitting length
    sumx, sumy = sum(vxs), sum(vys)
    sumxx, sumyy = sum([vx*vx for vx in vxs]), sum([vy*vy for vy in vys])
    varx, vary = sumxx/fl - sumx*sumx/(fl*fl), sumyy/fl - sumy*sumy/(fl*fl)
    sigma_vx, sigma_vy = np.sqrt(varx), np.sqrt(vary)
    
    return sigma_vx, sigma_vy


def method2(vxs, vys):
    return np.std(vxs), np.std(vys)


vx_data = np.linspace(1, 100, 5)
vy_data = np.linspace(5, 12, 5)
print(method1(vx_data, vy_data))
print(method2(vx_data, vy_data))
