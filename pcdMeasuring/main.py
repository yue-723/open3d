import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import Menu, filedialog

def pick_points(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  
    vis.destroy_window()

    return vis.get_picked_points()

def cal_distance(p1, p2):
    dis = 0
    for i in range(3):
        dis += (p1[i] - p2[i])**2
    return dis**0.5

def startButton_Click():
    filePath = filePath_label_text['text']
    if filePath == "":
        statusUpdate("Please select file !")
        return
    filePath = os.path.relpath(filePath) # 轉成相對路徑
    pcd = o3d.io.read_point_cloud(filePath)
    picked_id = pick_points(pcd)
    statusUpdate("")
    if len(picked_id) == 2:
        point_cloud_array = np.asarray(pcd.points)
        print(len(point_cloud_array))
        p1, p2 = point_cloud_array[picked_id[0]], point_cloud_array[picked_id[1]]
        Distance_label_text['text'] = f'{cal_distance(p1, p2)}'
        print(f'Distance : {cal_distance(p1, p2)}')

def menuLoad_Click():
    filePath = filedialog.askopenfilename(parent=root, initialdir='./', filetypes = (("ply files","*.ply"),("all files","*.*")))
    filePath_label_text['text'] = filePath

def statusUpdate(s):
    Status_label_text['text'] = s

root = tk.Tk()
root.title('選點測距')
root.geometry('400x100')
filePath = ""
# Create Menu
menuBar = tk.Menu(root)
fileMenu = tk.Menu(menuBar)
fileMenu = Menu(menuBar, tearoff=0)
fileMenu.add_command(label="Load", command=menuLoad_Click)
menuBar.add_cascade(label="File", menu=fileMenu)
root.config(menu=menuBar)
# Create Label
filePath_label = tk.Label(root, text=' 檔案位置 :  ', font=('Microsoft JhengHei UI', 10, 'bold'))
filePath_label.place(x=5, y=7)
filePath_label_text = tk.Label(root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
filePath_label_text.place(x=80, y=7)
Distance_label = tk.Label(root, text=' Distance :  ', font=('Microsoft JhengHei UI', 10, 'bold'))
Distance_label.place(x=5, y=37)
Distance_label_text = tk.Label(root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
Distance_label_text.place(x=75, y=37)
Status_label = tk.Label(root, text=' Status :  ', font=('Microsoft JhengHei UI', 10, 'bold'))
Status_label.place(x=5, y=67)
Status_label_text = tk.Label(root, text='', font=('Microsoft JhengHei UI', 10, 'bold'))
Status_label_text.place(x=67, y=67)

# Create Button
loadButton = tk.Button(root, command=startButton_Click, text="Start", font=('Microsoft JhengHei UI', 9, 'bold'),background='white smoke')
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
        
