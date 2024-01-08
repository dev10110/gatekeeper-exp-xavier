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

    def plot(self, ax, onlyObstacles=True):
        
        # get the right axes
        xs, ys = self.get_axes()

        # rescale the data to show only obstacles
        if onlyObstacles:
            data = 1.0 * self.data
            data[data > 0.075] = 1
            data[data <= 0.075] = 0
        else:
            data = self.data

        # make the plot
        # cmap =  mpl.colormaps['RdYlGn']
        ax.pcolorfast(xs, ys, data, cmap=cm)


        return


class BagProcessor:

    def __init__(self, filename):
        self.filename = filename;


    def list_topics(self):
        # create reader instance and open for reading
        with Reader(self.filename) as reader:
            for connection in reader.connections:
                print(connection.topic, connection.msgtype)

        return

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

                maps.append(sliceMap)
                stamps.append(sliceMap.timestamp)


        return stamps, maps


    def animate_maps(self, mode="certified", xlims=[-7,7], ylims=[-7,7]):

        stamps, maps = self.extract_maps("uncertified")
        cert_stamps, cert_maps = self.extract_maps("certified")
        N = min(len(maps), len(cert_maps))

        # create a window
        fig, (ax1, ax2)= plt.subplots(nrows=1, ncols=2);
        ax1.set_xlim(xlims)
        ax1.set_ylim(ylims)
        ax2.set_xlim(xlims)
        ax1.set_ylim(ylims)
        plt.show(block=False)


        for i in range(N):

            # get the maps
            m1 = maps[i]
            m2 = cert_maps[i] # hack

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
            ax1.set_title(f"{m1.timestamp:.0f}")
            ax2.set_title(f"{m2.timestamp:.0f}")

            # draw
            fig.canvas.draw()
            fig.canvas.flush_events()

        # block

        print("done animating")
        plt.show()

        return





if __name__== "__main__":

    filename = "/workspaces/isaac_ros-dev/rosbags/2024_01_08-11_42_52__run0_maps"
    filename = "/workspaces/isaac_ros-dev/rosbags/2024_01_08-11_48_09__run0_maps_clean"

    proc = BagProcessor(filename)

    proc.animate_maps();


