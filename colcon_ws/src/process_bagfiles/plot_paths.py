import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pickle 
import transforms3d

## first plt
fig = plt.figure()

ax_3d = fig.add_subplot(1, 4, 1, projection='3d')

df_true = pd.read_pickle("tfs_true.pkl")
df_noisy= pd.read_pickle("tfs_noisy.pkl")

ax_3d.plot(df_true.x, df_true.y, df_true.z)
ax_3d.plot(df_noisy.x, df_noisy.y, df_noisy.z)

ax_x = fig.add_subplot(1, 4, 2)
ax_y = fig.add_subplot(1, 4, 3)
ax_z = fig.add_subplot(1, 4, 4)


ax_x.plot(df_true.timestamp, df_true.x)
ax_y.plot(df_true.timestamp, df_true.y)
ax_z.plot(df_true.timestamp, df_true.z)

ax_x.plot(df_noisy.timestamp, df_noisy.x)
ax_y.plot(df_noisy.timestamp, df_noisy.y)
ax_z.plot(df_noisy.timestamp, df_noisy.z)



# second plot (rotations)

fig, axs = plt.subplots(2,2)
axs[0,0].plot(df_true.timestamp, df_true.qx)
axs[0,1].plot(df_true.timestamp, df_true.qy)
axs[1,0].plot(df_true.timestamp, df_true.qz)
axs[1,1].plot(df_true.timestamp, df_true.qw)

axs[0,0].plot(df_noisy.timestamp, df_noisy.qx)
axs[0,1].plot(df_noisy.timestamp, df_noisy.qy)
axs[1,0].plot(df_noisy.timestamp, df_noisy.qz)
axs[1,1].plot(df_noisy.timestamp, df_noisy.qw)

axs[0,0].set_title("qx")
axs[0,1].set_title("qy")
axs[1,0].set_title("qz")
axs[1,1].set_title("qw")


# third plot (euler)
fig, axs = plt.subplots(1, 3)
for df in [df_true, df_noisy]:
    axs[0].plot(df.timestamp, df.roll)
    axs[1].plot(df.timestamp, df.pitch)
    axs[2].plot(df.timestamp, df.yaw)

plt.show()

