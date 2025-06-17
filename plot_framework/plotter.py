import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

def animate(states, dt, xlim=None, ylim=None):

    fig, axs = plt.subplots(1, 3, figsize=(12, 4))
    time = np.linspace(0, dt * len(states), len(states))
    xdata, ydata = [], []

    def init():
        axs[0].set_title("x(t)")
        axs[1].set_title("y(t)")
        axs[2].set_title("x vs y")
        if xlim:
            axs[0].set_xlim(0, time[-1])
            axs[0].set_ylim(*xlim)
            axs[2].set_xlim(*xlim)
        if ylim:
            axs[1].set_xlim(0, time[-1])
            axs[1].set_ylim(*ylim)
            axs[2].set_ylim(*ylim)
        return axs

    def update(frame):
        x, y = states[frame]
        xdata.append(x)
        ydata.append(y)

        axs[0].cla()
        axs[1].cla()
        axs[2].cla()

        axs[0].plot(time[:frame], [s[0] for s in states[:frame]])
        axs[0].set_title("x(t)")
        axs[1].plot(time[:frame], [s[1] for s in states[:frame]])
        axs[1].set_title("y(t)")
        axs[2].plot(xdata, ydata)
        axs[2].set_title("x vs y")

        if xlim:
            axs[0].set_ylim(*xlim)
            axs[2].set_xlim(*xlim)
        if ylim:
            axs[1].set_ylim(*ylim)
            axs[2].set_ylim(*ylim)

        axs[0].set_xlim(0, time[-1])
        axs[1].set_xlim(0, time[-1])

        return axs

    ani = FuncAnimation(fig, update, frames=len(states), init_func=init, blit=False, interval=1)
    plt.tight_layout()
    plt.show()
