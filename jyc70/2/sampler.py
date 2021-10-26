import numpy as np

def sample():
    x = np.random.randint(101)/10.
    y = np.random.randint(101)/10.
    theta = np.random.uniform(-1,1) * np.pi
    return (x, y, theta)
