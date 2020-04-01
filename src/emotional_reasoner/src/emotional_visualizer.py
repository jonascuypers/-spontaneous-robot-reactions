#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from emotional_reasoner.msg import Emotion
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro')
rospy.init_node('emotional_visualizer', anonymous=True)


def visualize(data):
    update([data.valence, data.arousal])


rospy.Subscriber('emotional_reasoner', Emotion, visualize)


def init():
    circle = plt.Circle((0, 0), 1, color='black', fill=False)
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['top'].set_color('none')
    ax.spines['right'].set_color('none')
    ax.yaxis.tick_left()
    ax.xaxis.tick_bottom()
    ax.add_artist(circle)
    ax.xaxis.set_label_coords(0.95, 0.55)
    ax.yaxis.set_label_coords(0.5, 1)
    ax.set_xlim([-1.4, 1.4])
    ax.set_ylim([-1.4, 1.4])
    plt.xlabel("Valence")
    plt.ylabel("Arousal")
    return ln,


def update(frame):
    if type(frame) == list:
        xdata.append(frame[0])
        ydata.append(frame[1])
        ln.set_data(xdata, ydata)
    return ln,


ani = FuncAnimation(fig, update, init_func=init, blit=True)
plt.show()
