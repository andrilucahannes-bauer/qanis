import numpy as np
from plotter import animate
from system import SimpleOscillator, HopfOscillator


def run_simulation(system, steps=1000):
    states = []
    for _ in range(steps):
        states.append(system.step().copy())
    return np.array(states)

if __name__ == "__main__":
    osc = HopfOscillator(alpha=2.0, mu=1.0, omega=2.0, initial_state=[1.0, 0.0])
    states = run_simulation(osc, steps=1000)
    animate(states, dt=osc.dt, xlim=(-2, 2), ylim=(-2, 2))
