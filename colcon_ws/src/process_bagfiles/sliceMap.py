import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl


# create a colormap
from matplotlib.colors import LinearSegmentedColormap
cm = LinearSegmentedColormap.from_list('defcol', ["#FF0000", "#00FF00"])

class SliceMap:

    def __init__(self, data, origin_x, origin_y, width, height, resolution, timestamp=None, frame_id=None):
        self.data = data
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.width = width
        self.height = height
        self.resolution = resolution
        self.timestamp = timestamp
        self.frame_id = frame_id

    @classmethod
    def from_msg(cls, msg):
        
        img = np.array(msg.data).reshape([msg.height, msg.width])

        # fix the nanvalues
        img[img == msg.unknown_value] = np.nan

        # timestamp
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # save to new object
        return cls(img, msg.origin.x, msg.origin.y, msg.width, msg.height, msg.resolution, stamp, msg.header.frame_id)

    def get_axes(self):

        xs = self.origin_x + np.arange(self.width) * self.resolution
        ys = self.origin_y + np.arange(self.height) * self.resolution

        return xs, ys

    def plot(self, ax, onlyObstacles=True, transform=None):
        
        # get the right axes
        xs, ys = self.get_axes()

        if transform is not None:
            # assume we have a dictionary of the transform to use when plotting
            # it should contain ["x", "y", "yaw"] 
            tx = transform["x"]
            ty = transform["y"]
            theta = -np.pi/2  + transform["yaw"]
            
            # get coordinates of each point
            xx, yy = np.meshgrid(xs, ys)

            # transform each point
            xr = xx * np.cos(theta) + yy * np.sin(theta) +  tx
            yr = -xx * np.sin(theta) + yy * np.cos(theta) + ty

        else:
            xr, yr = np.meshgrid(xs, ys)

        # rescale the data to show only obstacles
        if onlyObstacles:
            data = 1.0 * self.data
            data[data > 0.075] = 1
            data[data <= 0.075] = 0
        else:
            data = self.data

        # make the plot
        # cmap =  mpl.colormaps['RdYlGn']
        ax.pcolormesh(xr, yr, data, cmap=cm, rasterized=True)
        ax.set_title(f"{self.timestamp:.1f}")
        ax.set_xlim([-8, 8])
        ax.set_ylim([-8, 8])


        return


