from rich.progress import track
import time
import itertools

x = range(7)
y = range(7)
r = range(7)

states = list(itertools.product(x, y, r))
n_states = len(states)

print(type(states), n_states, states)

for state in track(states, description="Executing"):
    print("Step: ", state)
    time.sleep(0.24)