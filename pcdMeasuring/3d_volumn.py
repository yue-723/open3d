from copyreg import pickle
import os
import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import Menu, filedialog


def pick_points(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(window_name='SSI_OCT PickCenter', width=1080,
                      height=720, visible=True)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    return vis.get_picked_points()


# def cal_distance(p1, p2):
#     dis = 0
#     for i in range(3):
#         dis += (p1[i] - p2[i])**2
#     return dis**0.5

def cal_SphereVolume(pcd, center):
    radius = int(InputRadius_text.get(1.0, tk.END+"-1c"))
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.colors[center] = [1, 0, 0]
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    [k, idx, _] = pcd_tree.search_radius_vector_3d(
        pcd.points[center], radius)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name='SSI_OCT', width=1080,
                      height=720, visible=True)
    vis.add_geometry(pcd)
    vis.run()
    vis.destroy_window()

    volumn = len(idx)
    print(f'Volumn : {volumn}')
    TotalVolumn_label_text['text'] = len(pcd.points)
    Center_label_text['text'] = pcd.points[center]
    Volumn_label_text['text'] = volumn
    statusUpdate("Calculate volumn successful")


def startButton_Click():
    filePath = filePath_label_text['text']
    if filePath == "":
        statusUpdate("Please select file !")
        return
    elif InputRadius_text.get(1.0, tk.END+"-1c") == "":
        statusUpdate("Please input radius !")
        return
    filePath = os.path.relpath(filePath)  # 轉成相對路徑
    pcd = o3d.io.read_point_cloud(filePath)
    picked_id = pick_points(pcd)
    statusUpdate("")
    if len(picked_id) == 1:
        cal_SphereVolume(pcd, picked_id[0])


def menuLoad_Click():
    filePath = filedialog.askopenfilename(
        parent=root, initialdir='./', filetypes=(("ply files", "*.ply"), ("all files", "*.*")))
    filePath_label_text['text'] = filePath


def statusUpdate(s):
    Status_label_text['text'] = s


root = tk.Tk()
root.title('3D Volumn Calculator')
root.geometry('550x200')
filePath = ""
# Create Menu
menuBar = tk.Menu(root)
fileMenu = tk.Menu(menuBar)
fileMenu = Menu(menuBar, tearoff=0)
fileMenu.add_command(label="Load", command=menuLoad_Click)
menuBar.add_cascade(label="File", menu=fileMenu)
root.config(menu=menuBar)
# Create Label
filePath_label = tk.Label(root, text=' File Path :  ', font=(
    'Microsoft JhengHei UI', 10, 'bold'))
filePath_label.place(x=5, y=7)
filePath_label_text = tk.Label(
    root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
filePath_label_text.place(x=80, y=7)
SetRadius_label = tk.Label(root, text=' Set Radius :  ',
                           font=('Microsoft JhengHei UI', 10, 'bold'))
SetRadius_label.place(x=5, y=37)
InputRadius_text = tk.Text(root, height=1, width=7)
InputRadius_text.place(x=90, y=40)
Center_label = tk.Label(root, text=' Picked Center :  ',
                        font=('Microsoft JhengHei UI', 10, 'bold'))
Center_label.place(x=5, y=67)
Center_label_text = tk.Label(
    root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
Center_label_text.place(x=105, y=67)
TotalVolumn_label = tk.Label(root, text=' Total volumn :  ',
                             font=('Microsoft JhengHei UI', 10, 'bold'))
TotalVolumn_label.place(x=5, y=97)
TotalVolumn_label_text = tk.Label(
    root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
TotalVolumn_label_text.place(x=100, y=97)
Volumn_label = tk.Label(root, text=' Volumn in specific sphere :  ',
                        font=('Microsoft JhengHei UI', 10, 'bold'))
Volumn_label.place(x=5, y=127)
Volumn_label_text = tk.Label(
    root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
Volumn_label_text.place(x=175, y=127)
Status_label = tk.Label(root, text=' Status :  ', font=(
    'Microsoft JhengHei UI', 10, 'bold'))
Status_label.place(x=5, y=157)
Status_label_text = tk.Label(root, text='', font=(
    'Microsoft JhengHei UI', 10, 'bold'))
Status_label_text.place(x=67, y=157)

# Create Button
loadButton = tk.Button(root, command=startButton_Click, text="Start", font=(
    'Microsoft JhengHei UI', 9, 'bold'), background='white smoke')
loadButton.place(relx=0.9, rely=0.75, anchor=tk.CENTER)


if __name__ == "__main__":

    # Start
    root.mainloop()
    # pcd = o3d.io.read_point_cloud("./bunny.ply")

    # picked_id = pick_points(pcd)

    # if len(picked_id) == 2:
    #     point_cloud_array = np.asarray(pcd.points)
    #     print(len(point_cloud_array))
    #     p1, p2 = point_cloud_array[picked_id[0]], point_cloud_array[picked_id[1]]
    #     print(f'Distance : {cal_distance(p1, p2)}')
