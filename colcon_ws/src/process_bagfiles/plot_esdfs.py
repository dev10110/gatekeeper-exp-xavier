import os
import glob
from pathlib import Path
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pickle
import pandas as pd


from sliceMap import SliceMap


def inverse_transform(tf):

    x = tf["x"]
    y = tf["y"]
    yaw = tf["yaw"] # assumed radians


    newYaw = -yaw
    newX = y * np.sin(yaw) - x * np.cos(yaw)
    newY = -x * np.sin(yaw) - y * np.cos(yaw)

    return {
            "x": newX,
            "y": newY,
            "yaw": newYaw
            }



class PlotESDFs():

    def __init__(self):
        
        self.load_tfs()

        self.load_slices()
        self.get_stamps()
        # self.correct_stamps()

        self.min_t, self.max_t = self.get_stamp_range()

        print(self.min_t, self.max_t)

        self.fig, self.axs = plt.subplots(2, 2)

    def load_tfs(self):

        self.gt_tf_true = pd.read_pickle("gt_tfs_true.pkl")
        self.gt_tf_pert = pd.read_pickle("gt_tfs_perturbed.pkl")
        self.noisy_tf_true = pd.read_pickle("noisy_tfs_true.pkl")
        self.noisy_tf_pert = pd.read_pickle("noisy_tfs_perturbed.pkl")

    def load_slices(self, show=False):
        
        self.gt_esdf = []
        self.gt_cesdf = []
        self.noisy_esdf = []
        self.noisy_cesdf = []

        for filepath in Path("./gt_map_slices").glob("esdf_*.pkl"):
            if show:
                print(filepath)
            with open(filepath, "rb") as f:
                s = pickle.load(f)
                self.gt_esdf.append(s)
        
        for filepath in Path("./gt_map_slices").glob("certified_esdf_*.pkl"):
            if show:
                print(filepath)
            with open(filepath, "rb") as f:
                s = pickle.load(f)
                self.gt_cesdf.append(s)
        
        for filepath in Path("./noisy_map_slices").glob("esdf_*.pkl"):
            if show:
                print(filepath)
            with open(filepath, "rb") as f:
                s = pickle.load(f)
                self.noisy_esdf.append(s)
        
        for filepath in Path("./noisy_map_slices").glob("certified_esdf_*.pkl"):
            if show:
                print(filepath)
            with open(filepath, "rb") as f:
                s = pickle.load(f)
                self.noisy_cesdf.append(s)

        print("done loading esdfs")
        
        self.gt_esdf.sort(key=lambda m: m.timestamp)
        self.gt_cesdf.sort(key=lambda m: m.timestamp)
        self.noisy_esdf.sort(key=lambda m: m.timestamp)
        self.noisy_cesdf.sort(key=lambda m: m.timestamp)
        print("done sorting esdfs")


    def get_stamps(self):
        

        self.stamp_gt_esdf = np.array([])
        self.stamp_gt_cesdf = np.array([])
        self.stamp_noisy_esdf = np.array([])
        self.stamp_noisy_cesdf = np.array([])

       
        for m in self.gt_esdf:
            self.stamp_gt_esdf = np.append(self.stamp_gt_esdf, m.timestamp)
        for m in self.gt_cesdf:
            self.stamp_gt_cesdf = np.append(self.stamp_gt_cesdf, m.timestamp)
        for m in self.noisy_esdf:
            self.stamp_noisy_esdf = np.append(self.stamp_noisy_esdf, m.timestamp)
        for m in self.noisy_cesdf:
            self.stamp_noisy_cesdf = np.append(self.stamp_noisy_cesdf, m.timestamp)

        print("done getting timestamps")


    def correct_stamps(self):

        min_t = np.min(self.stamp_gt_esdf)
        print(min_t)
        for m in self.gt_esdf:
            m.timestamp = m.timestamp - min_t

        for m in self.gt_cesdf:
            m.timestamp = m.timestamp - min_t
        

        min_t = np.min(self.stamp_noisy_esdf)
        print(min_t)
        for m in self.noisy_esdf:
            m.timestamp = m.timestamp - min_t
        
        for m in self.noisy_cesdf:
            m.timestamp = m.timestamp - min_t

        self.get_stamps()


    def get_stamp_range(self):

        min_t = np.min(self.stamp_gt_esdf)
        min_t = np.minimum(min_t,np.min(self.stamp_gt_cesdf))
        min_t = np.minimum(min_t,np.min(self.stamp_noisy_esdf))
        min_t = np.minimum(min_t,np.min(self.stamp_noisy_cesdf))
        
        max_t = np.max(self.stamp_gt_esdf)
        max_t = np.maximum(max_t,np.max(self.stamp_gt_cesdf))
        max_t = np.maximum(max_t,np.max(self.stamp_noisy_esdf))
        max_t = np.maximum(max_t,np.max(self.stamp_noisy_cesdf))

        return min_t, max_t


    # def do_plots(self, t):


    #     # get the time index for the slices
    #     gt_esdf_i = np.searchsorted(self.stamp_gt_esdf, t) 
    #     gt_cesdf_i = np.searchsorted(self.stamp_gt_cesdf, t)
    #     noisy_esdf_i = np.searchsorted(self.stamp_noisy_esdf, t)
    #     noisy_cesdf_i = np.searchsorted(self.stamp_noisy_cesdf, t)


    #     # plot the slice
    #     self.gt_esdf[gt_esdf_i].plot(self.axs[0,0])
    #     self.gt_cesdf[gt_cesdf_i].plot(self.axs[0,1])
    #     self.noisy_esdf[noisy_esdf_i].plot(self.axs[1,0])
    #     self.noisy_cesdf[noisy_cesdf_i].plot(self.axs[1,1])






        # plt.show()

    def plot_fov(self, ax, tf=None, L = 5.0, fov = 90 * np.pi / 180 ):

        # ignores the z component
        if tf is None:
            x0 = 0.0
            y0 = 0.0
            yaw = np.pi/2
        else:
            x0 = tf["x"]
            y0 = tf["y"]
            yaw = tf["yaw"]

        x1 = x0 + L*np.cos(yaw + fov/2)
        y1 = y0 + L*np.sin(yaw + fov/2)
        x2 = x0 + L*np.cos(yaw - fov/2)
        y2 = y0 + L*np.sin(yaw - fov/2)

        ax.plot([x0, x1,x2,x0], [y0, y1, y2, y0])


    def animate(self, N):
        t_min, t_max = self.get_stamp_range()

        for t in np.linspace(t_min, t_max, N):
            self.do_plots(t)
            plt.pause(0.1)

    def save_all(self):

        N_gt = len(self.stamp_gt_esdf)
        N_ns = len(self.stamp_noisy_esdf)

        if (N_gt != N_ns):
            print("not consistent!!")
            return

        # delete all exisiting plots
        files = glob.glob("./plts/*")
        for f in files:
            os.remove(f)

        for i in range(N_gt):
            
            # clear the axes
            # self.axs[0,0].clear()
            # self.axs[0,1].clear()
            # self.axs[1,0].clear()
            self.axs[1,1].clear()
            
            # get the timestamp
            current_t = self.gt_esdf[i].timestamp

            # get the tf_row
            ts = self.gt_tf_true["timestamp"]
            tf_gt_true_i = np.searchsorted(self.gt_tf_true["timestamp"], current_t)
            tf_gt_pert_i = np.searchsorted(self.gt_tf_pert["timestamp"], current_t)
            tf_noisy_true_i = np.searchsorted(self.noisy_tf_true["timestamp"], current_t)
            tf_noisy_pert_i = np.searchsorted(self.noisy_tf_pert["timestamp"], current_t)

            tf_gt_true = self.gt_tf_true.iloc[tf_gt_true_i]
            tf_gt_pert = self.gt_tf_pert.iloc[tf_gt_pert_i]
            tf_noisy_true = self.noisy_tf_true.iloc[tf_noisy_true_i]
            tf_noisy_pert = self.noisy_tf_pert.iloc[tf_noisy_pert_i]

            # print(tf_gt_true)
            

            # plot the slices in vicon/world
            # self.gt_esdf[i].plot(self.axs[0,0])
            # self.gt_cesdf[i].plot(self.axs[0,1], transform=(tf_gt_true))
            # self.noisy_esdf[i].plot(self.axs[1,0], transform=(tf_noisy_true))
            self.noisy_cesdf[i].plot(self.axs[1,1])
            
            # plot the slices in body-fixed frame
            # self.gt_esdf[i].plot(self.axs[0,0], transform=(tf_gt_true))
            # self.gt_cesdf[i].plot(self.axs[0,1], transform=(tf_gt_true))
            # self.noisy_esdf[i].plot(self.axs[1,0], transform=(tf_noisy_true))
            # self.noisy_cesdf[i].plot(self.axs[1,1], transform=(tf_noisy_true))


            # plot the fov
            # self.plot_fov(self.axs[0,0], inverse_transform(self.gt_tf_true.iloc[tf_gt_true_i]))
            # self.plot_fov(self.axs[0,1], self.gt_tf_pert.iloc[tf_gt_pert_i])
            # self.plot_fov(self.axs[1,0], self.noisy_tf_true.iloc[tf_noisy_true_i])
            self.plot_fov(self.axs[1,1], self.noisy_tf_pert.iloc[tf_noisy_pert_i])
            self.plot_fov(self.axs[1,1], self.noisy_tf_true.iloc[tf_noisy_true_i])
            
            # self.plot_fov(self.axs[0,0])
            # self.plot_fov(self.axs[0,1])
            # self.plot_fov(self.axs[1,0])
            # self.plot_fov(self.axs[1,1])

            plt.savefig(f"./plts/{i:05d}.png")
            print(f"saving {i}/{N_gt}")










if __name__ == "__main__":
    plotter = PlotESDFs()
    plotter.save_all()

    
