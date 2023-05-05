
import numpy as np
minn = 1000
very_i = 0
for i in range(2000):
    x = i*np.pi/1000
    y = 4**(np.sin(x)**2) + 4**(np.cos(x)**2)
    diff = abs(4-y)
    if diff < minn:
        minn = diff
        very_i = i
    print(i)
print('very: ', very_i)
'''
x^2代表x的平方。
解：令(sinx)^2 = t, t取值范围是[0,1]
则（cosx)^2 = 1-t
4^t+4^(1-t) = 4
4*(4^(t-1))+4*(4^(-t)) = 4
4^(t-1)+4^(-t) = 1
4^t/4 + 1/4^t = 1
令4^t = u，u的取值范围是[0,+inf)
u/4 + 1/u = 1, 等式两边乘以4*u得
u^2 + 4 = 4*u, 即u^2-4*u+4 = 0，即
(u-2)^2 = 0, 即 u= 2,
即4^t = 2, t = 0.5
(sinx)^2 = 0.5, sinx = (+-)sqrt(2)/2
x = pi/4 或 3*pi/4 或 5*pi/4 或 7*pi/4




'''






