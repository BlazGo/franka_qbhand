iterations = 0

class Environment:
    def __init__(self, shape):
        self.shape = shape
        self.visited = set()

    def is_valid_state(self, state):
        return all(0 <= state[i] < self.shape[i] for i in range(3))

    def get_neighbors(self, state):
        neighbors = []
        for i in range(3):
            for delta in [-1, 1]:
                new_state = list(state)
                new_state[i] += delta
                if self.is_valid_state(new_state):
                    neighbors.append(tuple(new_state))
        return neighbors

    def dfs(self, state):
        global iterations
        iterations += 1
        if state in self.visited:
            return
        self.visited.add(state)
        for neighbor in self.get_neighbors(state):
            self.dfs(neighbor)

    def visit_all_states(self):
        self.dfs((0, 0, 0))
        return len(self.visited) == self.shape[0] * self.shape[1] * self.shape[2]


# Test the program
env = Environment([7, 7, 5])
print(env.visit_all_states())  # Should print True
print(iterations, 7*7*5)