#%%
import matplotlib.pyplot as plt
import matplotlib as mpl
import numpy as np


x = np.linspace(0, 20, 100)
traj=np.sin(x)
plt.plot(x, np.sin(x))
plt.show()
