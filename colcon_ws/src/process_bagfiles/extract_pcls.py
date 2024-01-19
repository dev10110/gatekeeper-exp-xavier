# needs rosbags
# python3 -m pip install rosbags

from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
# from nvblox_msgs.msg import DistanceMapSlice
# from rosbags.typesys.types import nvblox_msgs__msg__DistanceMapSlice as DistanceMapSlice
from rosbags.typesys import get_types_from_msg, register_types

from pathlib import Path
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import pickle
import json
import h5py

import pandas as pd
import transforms3d 
import glob
import os

from sliceMap import SliceMap


from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField


def guess_msgtype(path: Path) -> str:
    """Guess message type name from path."""
    name = path.relative_to(path.parents[2]).with_suffix('')
    if 'msg' not in name.parts:
        name = name.parent / 'msg' / name.name
    return str(name)

def register_all_types():
    
    add_types = {}
    
    for pathstr in [
        '/workspaces/isaac_ros-dev/src/isaac_ros_nvblox/nvblox_msgs/msg/DistanceMapSlice.msg',
    ]:
        msgpath = Path(pathstr)
        msgdef = msgpath.read_text(encoding='utf-8')
        add_types.update(get_types_from_msg(msgdef, guess_msgtype(msgpath)))
    
    
    register_types(add_types)


# first register the types
register_all_types()





def get_group(hf, n):

    try:
        g = hf.create_group(n)
        return g
    except Exception as e:
        return hf[n]



class BagProcessor:

    def __init__(self, filename, isClean): # isClean refers to whether there is noise on the vicon data
        self.isClean = isClean;
        self.filename = filename;

        self.list_topics()
        
        # extract tfs
        self.extract_tfs()
        self.save_tfs()

        self.extract_and_save_pcls(mode="uncertified")
        self.extract_and_save_pcls(mode="certified")

        # extract maps
        #self.extract_maps("uncertified")
        #self.extract_maps("certified")


        # correct timestamps based on first published map
        # self.correct_stamps()

        # if "clean" in self.filename:
        #     # self.save_maps("./gt_map_slices")
        #     # self.save_tfs("./gt_tfs")
        #     self.to_json("./clean_slices_json")
        # if "noisy" in self.filename:
        #     # self.save_maps("./noisy_map_slices")
        #     # self.save_tfs("./noisy_tfs")
        #     self.to_json("./noisy_slices_json")


    def list_topics(self):
        # create reader instance and open for reading
        with Reader(self.filename) as reader:
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

        return

    def extract_and_save_pcls(self, mode, path="."):

        fn = f"{path}/pcls.h5"
        print(f"saving to {fn}")

        # open up the data
        with h5py.File(fn, 'a') as hf:
            
            if self.isClean:
                n = f"/pcl/clean/{mode}"
            else:
                n = f"/pcl/noisy/{mode}"

            g = hf.create_group(n)

            print(f"created group {n}")

            if mode == "uncertified":
                topic = "/nvblox_node/esdf_pointcloud"
            else:
                topic = "/nvblox_node/certified_esdf_pointcloud"

            with Reader(self.filename) as reader:

                connections = [x for x in reader.connections if x.topic==topic]

                index = 0

                for connection, timestamp, rawdata in reader.messages(connections=connections):

                    print(index, timestamp) 

                    msg = deserialize_cdr(rawdata, connection.msgtype)

                    # create the pointcloud
                    pc = PointCloud2()
                    pc.header.stamp.sec = msg.header.stamp.sec
                    pc.header.stamp.nanosec = msg.header.stamp.nanosec
                    pc.header.frame_id = msg.header.frame_id
                    pc.height = msg.height
                    pc.width = msg.width
                    pc.fields = []
                    for field in msg.fields:
                        f = PointField()
                        f.name = field.name
                        f.offset = field.offset
                        f.datatype = field.datatype
                        f.count = field.count
                        pc.fields.append(f)
                    pc.is_bigendian = msg.is_bigendian
                    pc.point_step = msg.point_step
                    pc.row_step = msg.row_step
                    pc.data = [ int(x) for x in msg.data]
                    pc.is_dense = msg.is_dense

                    # convert it into data
                    data = point_cloud2.read_points(pc)

                    # convert it into a plain numpy array
                    arr = np.vstack([
                        data["x"], 
                        data["y"], 
                        data["z"],
                        data["intensity"]])

                    # add it to the hdf5 file
                    ds = g.create_dataset(f"{timestamp}", data=arr)

                    index += 1
                    # if index > 20:
                    #     break

            hf.close()



    def extract_tfs(self, mode="true"):

        self.tf_true = []
        self.tf_perturbed = []

        with Reader(self.filename) as reader:

            connections = [x for x in reader.connections if x.topic == "/tf"]

            for connection, timestamp, rawdata in reader.messages(connections=connections):

                tf_msg = deserialize_cdr(rawdata, connection. msgtype)
                
                for msg in tf_msg.transforms:

                    msg_dict = {
                            # "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            "timestamp": timestamp,
                            "frame_id": msg.header.frame_id,
                            "child_frame_id": msg.child_frame_id,
                            "x": msg.transform.translation.x,
                            "y": msg.transform.translation.y,
                            "z": msg.transform.translation.z,
                            "qx": msg.transform.rotation.x,
                            "qy": msg.transform.rotation.y,
                            "qz": msg.transform.rotation.z,
                            "qw": msg.transform.rotation.w
                            }

                    # print(msg_dict)
                    if msg_dict["child_frame_id"] == "vicon/laptop_realsense_true/laptop_realsense_true":
                        self.tf_true.append(msg_dict)
                    if msg_dict["child_frame_id"] == "vicon/laptop_realsense/laptop_realsense":
                        self.tf_perturbed.append(msg_dict)

        print(f"got all the tf msgs, {len(self.tf_true)}, {len(self.tf_perturbed)}")
        self.df_true = pd.DataFrame(self.tf_true)
        self.df_perturbed = pd.DataFrame(self.tf_perturbed)

        # add in the roll pitch yaw
        for df in [self.df_true, self.df_perturbed]:

            r = []
            p = []
            y = []

            for index, row in df.iterrows():
                rr, pp, yy = transforms3d.euler.quat2euler([row["qw"], row["qx"], row["qy"], row["qz"]])
                r.append(rr)
                p.append(pp)
                y.append(yy)

            df["roll"] = r
            df["pitch"] = p
            df["yaw"] = np.unwrap(np.array(y))


    def save_tfs(self, path="./"):

        # save it
        # self.df_true.to_pickle(path + "_true.pkl")
        # self.df_perturbed.to_pickle(path + "_perturbed.pkl")

        # save it to the h5 file
        if self.isClean:
            n = "/tf/clean"
        else:
            n = "/tf/noisy"

        with h5py.File("./pcls.h5", "a") as hf:
                
            g = hf.create_group(n)

            for (df, label) in [(self.df_true, "true"), (self.df_perturbed, "perturbed")]:
                data = np.vstack([
                    df["timestamp"],
                    df["x"],
                    df["y"],
                    df["z"],
                    df["qw"],
                    df["qx"],
                    df["qy"],
                    df["qz"],
                    df["roll"],
                    df["pitch"],
                    df["yaw"],
                    ])

                
                g.create_dataset(label, data=data)

    def extract_maps(self, mode="certified"):
        

        print("extracting maps...")


        if mode == "uncertified":
            topic = "/nvblox_node/map_slice"

        if mode == "certified":
            topic = "/nvblox_node/certified_map_slice"


        stamps = []
        maps = []


        # create reader instance and open for reading
        with Reader(self.filename) as reader:
        
            # grab the map_slices and plot them
            connections = [x for x in reader.connections if x.topic == topic]
        
            for connection, timestamp, rawdata in reader.messages(connections=connections):

                # deserialize
                msg = deserialize_cdr(rawdata, connection.msgtype)
                
                # convert to my custom type 
                sliceMap = SliceMap.from_msg(msg)
                sliceMap.timestamp = timestamp * 1e-9

                maps.append(sliceMap)
                stamps.append(sliceMap.timestamp)

        stamps = np.array(stamps)

        if (mode=="uncertified"):
            self.stamps = stamps
            self.maps = maps

        if (mode=="certified"):
            self.cert_stamps = stamps
            self.cert_maps = maps


        return stamps, maps

    def save_maps(self, path="./map_slices"):

        Path(path).mkdir(parents=True, exist_ok=True)

        print(f"saving to {path}")


        # save the uncertified
        for i in range(len(self.stamps)):
            fn = f"{path}/esdf_{self.stamps[i]}.pkl"
            with open(fn, "wb") as f:
                pickle.dump(self.maps[i], f)

        # save the certified
        for i in range(len(self.cert_stamps)):
            fn = f"{path}/certified_esdf_{self.stamps[i]}.pkl"
            with open(fn, "wb") as f:
                pickle.dump(self.cert_maps[i], f)


        print("Saved esdfs")

    def to_json(self, path="./map_slices_json/clean"):

        os.makedirs(path, exist_ok=True)
        for f in glob.glob(f"{path}/*"):
            os.remove(f)

        
        # for each map convert it to a json object, and then save the corresponding tfs with it 

        for (i, m) in enumerate([self.maps, self.cert_maps]):
            for (j, s) in enumerate(m):
                # obj = vars(s)
                obj = {}
                obj["timestamp"] = s.timestamp
                obj["data"] = s.data.tolist()
                obj["origin_x"] = s.origin_x
                obj["origin_y"] = s.origin_y
                obj["width"] = s.width
                obj["heigth"] = s.height
                obj["resolution"] = s.resolution


                # get the appropriate tfs
                ind_true = np.searchsorted(self.df_true["timestamp"], s.timestamp)
                obj["tf_true"] = self.df_true.iloc[ind_true].to_dict()
                
                ind_noisy = np.searchsorted(self.df_noisy["timestamp"], s.timestamp)
                obj["tf_noisy"] = self.df_noisy.iloc[ind_noisy].to_dict()

                # dump
                if i == 0: # uncertified
                    fn = f"{path}/uncertified_{j:05d}.json"
                else:
                    fn = f"{path}/certified_{j:05d}.json"

                with open(fn, "w", encoding="utf-8") as f:
                    json.dump(obj, f, ensure_ascii=False, indent=2)



    
    def correct_stamps(self):

        self.start_stamp = min(self.stamps[0], self.cert_stamps[0])

        self.stamps = self.stamps - self.start_stamp
        self.cert_stamps = self.stamps - self.start_stamp

        # correct it in each slice
        for m in self.maps:
            m.timestamp = m.timestamp - self.start_stamp

        for m in self.cert_maps:
            m.timestamp = m.timestamp - self.start_stamp

    def get_map_count(self):

        return len(self.stamps), len(self.cert_stamps)



    def animate_maps(self, mode="certified", xlims=[-7,7], ylims=[-7,7]):

        N = min(len(self.maps), len(self.cert_maps))

        # create a window
        fig, (ax1, ax2)= plt.subplots(nrows=1, ncols=2);
        ax1.set_xlim(xlims)
        ax1.set_ylim(ylims)
        ax2.set_xlim(xlims)
        ax1.set_ylim(ylims)
        plt.show(block=False)


        for i in range(N):

            # get the maps
            m1 = self.maps[i]
            m2 = self.cert_maps[i] # hack

            # plot
            ax1.clear()
            ax2.clear()
            m1.plot(ax1)
            m2.plot(ax2)
            
            # axes
            ax1.set_xlim(xlims)
            ax1.set_ylim(ylims)
            ax2.set_xlim(xlims)
            ax2.set_ylim(ylims)

            # title
            ax1.set_title(f"{m1.timestamp:.2f}")
            ax2.set_title(f"{m2.timestamp:.2f}")

            # draw
            fig.canvas.draw()
            fig.canvas.flush_events()

        # block

        print("done animating")
        plt.show()

        return





if __name__== "__main__":

    filename = "/workspaces/isaac_ros-dev/rosbags/mapping/run_1/run_1_clean_2024_01_16-11_48_24"
    proc_clean = BagProcessor(filename, True)
    
    filename = "/workspaces/isaac_ros-dev/rosbags/mapping/run_1/run_1_noisy_2024_01_16-11_39_39"
    proc_noisy = BagProcessor(filename, False)

