#!/usr/bin/env python
import matplotlib.pyplot as plt
import rospy
from matplotlib.animation import FuncAnimation
from emotional_reasoner.msg import Emotion

# This visualisation was hacked together to display the emotion experienced trough time
# My experience in visualisation is limited but to my knowledge this was not possible using rqt
fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'ro')
rospy.init_node('emotional_visualizer', anonymous=True)


def visualize(data):
    update([data.valence, data.arousal])


rospy.Subscriber('emotional_reasoner', Emotion, visualize)


def init():
    # Initialise the main circle for plotting emotions
    circle = plt.Circle((0, 0), 1, color='black', fill=False)
    # Put the axises in the middle
    ax.spines['left'].set_position('zero')
    ax.spines['bottom'].set_position('zero')
    ax.spines['top'].set_color('none')
    ax.spines['right'].set_color('none')
    ax.yaxis.tick_left()
    ax.xaxis.tick_bottom()
    ax.add_artist(circle)
    # Locate the labels the axes
    ax.xaxis.set_label_coords(0.95, 0.55)
    ax.yaxis.set_label_coords(0.5, 1)
    ax.set_xlim([-1.4, 1.4])
    ax.set_ylim([-1.4, 1.4])
    # Label the axes
    plt.xlabel("Valence")
    plt.ylabel("Arousal")
    return ln,


def update(frame):
    if type(frame) == list:
        xdata.append(frame[0])
        ydata.append(frame[1])
        # only show the last 5 points
        ln.set_data(xdata, ydata)
    return ln,


ani = FuncAnimation(fig, update, init_func=init, blit=True)
plt.show()
