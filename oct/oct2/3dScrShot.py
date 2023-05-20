import os
import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import Menu, filedialog
import matplotlib.pyplot as plt
from datetime import datetime


def startButton_Click():

    filePath = filePath_label_text['text']
    if filePath == "":
        statusUpdate("Please select file !")
        return
    filePath = os.path.relpath(filePath)  # 轉成相對路徑
    pcd = o3d.io.read_point_cloud(filePath)
    # vis = o3d.visualization.VisualizerWithEditing()
    vis = o3d.visualization.VisualizerWithKeyCallback()

    def key_action_callback(vis, action, mods):
        if action == 0:  # key up
            image = vis.capture_screen_float_buffer()
            dt_string = datetime.now().strftime("%Y%m%d-%H-%M-%S")
            file_path = filedialog.asksaveasfile(mode='w', initialdir='./test_data/',
                                                 initialfile=f"{dt_string}_ScrShot", defaultextension=".png",
                                                 filetypes=[("PNG File", "*.png"), ("JPG File", "*.jpg")])
            if file_path is None:
                return
            plt.imsave(str(file_path.name), np.asarray(image), dpi=1)
        # elif action == 1:  # key down
        # elif action == 2:  # key repeat
        return True

    # key_action_callback will be triggered when there's a keyboard press, release or repeat event
    vis.register_key_action_callback(ord("P"), key_action_callback)

    # animation_callback is always repeatedly called by the visualizer
    # vis.register_animation_callback(animation_callback)
    vis.create_window(window_name='SSI_OCT', width=1080,
                      height=960, visible=True)
    vis.add_geometry(pcd)
    view_ctl = vis.get_view_control()

    if radioValue.get() == 1:           # "Top"
        view_ctl.set_up((0, 1, 0))
    elif radioValue.get() == 2:         # "Bottom"
        view_ctl.set_front((0, 0, -5))
    elif radioValue.get() == 3:         # "Front"
        view_ctl.set_front((1, -5, 0))
    elif radioValue.get() == 4:         # "Back"
        view_ctl.set_front((-1, 5, 0))
    elif radioValue.get() == 5:         # "Left"
        view_ctl.set_front((-1, 0, 0))
    elif radioValue.get() == 6:         # "Right"
        view_ctl.set_front((1, 0, 0))
    elif radioValue.get() == 7:         # "ISO_VIEW_1"
        view_ctl.set_up((1, 1, 0))
        view_ctl.set_front((1, 1, 1))
    elif radioValue.get() == 8:         # "ISO_VIEW_2"
        view_ctl.set_front((0, 1, 1))

    vis.update_geometry(pcd)
    vis.poll_events()
    vis.update_renderer()
    vis.run()

    statusUpdate("")


def menuLoad_Click():
    filePath = filedialog.askopenfilename(
        parent=root, initialdir='./', filetypes=(("ply files", "*.ply"), ("all files", "*.*")))
    filePath_label_text['text'] = filePath


def statusUpdate(s):
    Status_label_text['text'] = s


root = tk.Tk()
root.title('3D ViewPoint ScrShot')
root.geometry('480x260')
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

# Vop = View of Point
SetVoP_label = tk.Label(root, text=' 相機視角 :  ',
                        font=('Microsoft JhengHei UI', 10, 'bold'))
SetVoP_label.place(x=5, y=37)

VoPs = {"Top": "1",
        "Bottom": "2",
        "Front": "3",
        "Back": "4",
        "Left": "5",
        "Right": "6",
        "ISO_VIEW_1": "7",
        "ISO_VIEW_2": "8"}
radioValue = tk.IntVar(None, 1)
for (text, value) in VoPs.items():
    if int(value) <= 4:
        tk.Radiobutton(root, text=text, value=value, font=(
            'Microsoft JhengHei UI', 10, 'bold'), variable=radioValue).place(x=5, y=(int(value) - 1)*30 + 67)
    else:
        tk.Radiobutton(root, text=text, value=value, font=(
            'Microsoft JhengHei UI', 10, 'bold'), variable=radioValue).place(x=105, y=(int(value) - 5)*30 + 67)

Status_label = tk.Label(root, text=' Status :  ', font=(
    'Microsoft JhengHei UI', 10, 'bold'))
Status_label.place(x=5, y=187)
Status_label_text = tk.Label(root, text='', font=(
    'Microsoft JhengHei UI', 10, 'bold'))
Status_label_text.place(x=67, y=187)

# Start Button
StartButton = tk.Button(root, command=startButton_Click, text="Start", font=(
    'Microsoft JhengHei UI', 9, 'bold'), background='white smoke')
StartButton.place(x=420, y=220)

if __name__ == "__main__":

    # Start
    root.mainloop()
