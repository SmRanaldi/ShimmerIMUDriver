import numpy as np
import pandas as pd
import quaternion
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import mpl_toolkits.mplot3d.axes3d as p3
import os

def visualize_3d(quaternion_in):

    vertices = np.array([[1.5,1.,0.],[1.5,1.,0.3],
                        [1.5,-1.,0.],[1.5,-1.,0.3],
                        [-1.5,-1.,0.],[-1.5,-1.,0.3],
                        [-1.5,1.,0.],[-1.5,1.,0.3]])
    vertices_all = list([vertices])

    for i in range(len(quaternion_in)):
        q = quaternion_in[i]
        tmp_vert = np.ones(vertices.shape)
        for j in range(vertices.shape[0]):
            tmp_quaternion = np.insert(np.array(vertices[j,:]),0,0.)
            tmp_quaternion = np.quaternion(*tmp_quaternion)
            tmp_quaternion = q * tmp_quaternion * q.conj()
            tmp_vert[j,:] = quaternion.as_float_array(tmp_quaternion)[1:]
        vertices_all.append(np.array(tmp_vert))

    n_frames = len(vertices_all)
    
    fig = plt.figure()
    ax = p3.Axes3D(fig)
    ax.set_xlim3d([-2,2])
    ax.set_ylim3d([-2,2])
    ax.set_zlim3d([-2,2])
    line = ax.scatter([],[],[], s=40, color='k')

    def init():
        line._offsets3d=[[],[],[]]
        return line
    def animate(i,points):
        print(f"Frame {i} of {n_frames}.")
        line._offsets3d=[points[i][:,0],points[i][:,1],points[i][:,2]]
        return line

    anim = FuncAnimation(fig, animate, init_func=init, fargs=(vertices_all,),
                            frames=len(quaternion_in), interval=5, repeat=False)
    print(f"{n_frames} frames.")
    plt.show()

if __name__ == "__main__":
    data = pd.read_csv("test_data/quaternions_Com5.csv")
    data = data.values[::5,1:]
    print(data.shape)
    quat = quaternion.from_float_array(data)
    visualize_3d(quat)