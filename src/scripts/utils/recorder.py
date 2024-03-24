#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import datetime
import io, json, math, os, time
from threading import Thread
from functools import partial
from tkinter import *

HOME_DIR = "/home/ubuntu"
CONFIG_DIR = HOME_DIR + "/catkin_ws/src/morai_drive/src/scripts/autonomous_driving/config"
SAVE_DIR = HOME_DIR + "/Desktop/record/"

class TimeRecorder:
    def __init__(self):
        self.reset()

    def start(self):
        self.start_time = time.time()
        self.is_start = True

    def stop(self):
        if(self.is_start == True):
            self.end_time = time.time()
            self.elapsed_time = self.end_time - self.start_time
            self.is_start = False
            return self.elapsed_time

    def reset(self):
        self.start_time = 0
        self.end_time = 0
        self.elapsed_time = 0
        self.is_start = False

class ConfigRecorder:
    def __init__(self):
        pass

    def save(self, filename, data):
        prefix = SAVE_DIR
        suffix = datetime.datetime.now().strftime('%y%m%d_%H%M%S')
        filename = prefix + suffix + ".json"
        with open(filename, 'w') as f:
            json.dump(data, f, indent=4)

def start(w):
    w.t.start()
    w.log(str("start"))
    id = time.strftime('%H%M%S', time.localtime(time.time()))
    t = Thread(target=update, args=(id, w), daemon=True)
    t.start()

def update(id, w):
    while True:
        if(w.t.is_start != True):
           return
        t = math.floor((time.time() - w.t.start_time) * 1000) / 1000
        w.update_time(t)

def stop(w):
    elapsed_time = w.t.stop()
    elapsed_time = math.floor(elapsed_time * 1000) / 1000
    w.label.config(text=str(elapsed_time))
    w.log("elapsed_time: "+str(elapsed_time))
    data = {"elapsed_time":elapsed_time}
    with io.open(os.path.join(CONFIG_DIR, 'config.json'), 'r', encoding='utf-8') as f:
        config = json.load(f)
        data.update(config)
    w.c.save("recorder" ,data)

def reset(w):
    w.t.reset()
    w.update_time(00.000)
    w.log("reset")

def clear(w):
    w.clear()

class Window:
    def __init__(self):
        # recorder
        self.t = TimeRecorder()
        self.c = ConfigRecorder()

        # tk
        self.tk=Tk()
        self.tk.wm_attributes("-topmost", 1) # always top level window
        self.tk.title('recorder')

        # record button
        btn = Button(self.tk, text="start", width=5,command=partial(start, self))
        btn.grid(row=0, column=0)
        btn = Button(self.tk, text="stop", width=5,command=partial(stop, self))
        btn.grid(row=0, column=1)
        btn = Button(self.tk, text="reset", width=5,command=partial(reset, self))
        btn.grid(row=0, column=2)
        btn = Button(self.tk, text="clear", width=5,command=partial(clear, self))
        btn.grid(row=0, column=3)

        # time
        self.label = Label(self.tk, text="00.000", width=10)
        self.label.grid(row=1, column=0)

        # log
        self.label_title = Label(self.tk, text='')
        self.label_title.grid(row=1, column=0, columnspan=5, sticky="w")
        self.scroll = Scrollbar(self.tk, orient='vertical')
        self.lbox = Listbox(self.tk, yscrollcommand=self.scroll.set, width=50)
        self.scroll.config(command=self.lbox.yview)
        self.lbox.grid(row=2, column=0, columnspan=5, sticky="s")

        # execute
        self.tk.mainloop()

    def log(self,msg):
        msg="["+time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))+"] "+msg
        self.lbox.insert(END, msg)
        self.lbox.update()
        self.lbox.see(END)
        print(msg)

    def clear(self):
        self.lbox.delete(0, END)
        self.log("clear..")

    def update_time(self, time):
        self.label.config(text=str(time))
w=Window()
