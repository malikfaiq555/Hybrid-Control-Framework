import random
from collections import deque
import numpy as np

class ReplayBuffer(object):
    """
    Experience Replay Buffer for Deep RL.
    Stores (state, action, reward, done, next_state) tuples.
    Implements fixed-size FIFO (first-in, first-out) with random sampling.
    """

    def __init__(self, buffer_size, random_seed=123):
        """
        Initialize the buffer.

        Args:
            buffer_size (int): Maximum number of transitions to store.
            random_seed (int): Seed for reproducibility.
        """
        self.buffer_size = buffer_size
        self.count = 0
        self.buffer = deque()
        random.seed(random_seed)

    def add(self, s, a, r, t, s2):
        """
        Add a new experience to the buffer.

        Args:
            s: Current state (numpy array).
            a: Action taken (numpy array or list).
            r: Reward (float).
            t: Done flag (1 if episode ended, else 0).
            s2: Next state (numpy array).
        """
        experience = (s, a, r, t, s2)
        if self.count < self.buffer_size:
            self.buffer.append(experience)
            self.count += 1
        else:
            # Buffer is full: remove oldest experience (FIFO)
            self.buffer.popleft()
            self.buffer.append(experience)

    def size(self):
        """Returns the current size of the buffer."""
        return self.count

    def sample_batch(self, batch_size):
        """
        Randomly sample a batch of experiences from the buffer.

        Args:
            batch_size (int): Number of samples to return.

        Returns:
            Tuple of (states, actions, rewards, dones, next_states), each as np.array.
        """
        if self.count < batch_size:
            batch = random.sample(self.buffer, self.count)
        else:
            batch = random.sample(self.buffer, batch_size)

        s_batch = np.array([_[0] for _ in batch])
        a_batch = np.array([_[1] for _ in batch])
        r_batch = np.array([_[2] for _ in batch]).reshape(-1, 1)
        t_batch = np.array([_[3] for _ in batch]).reshape(-1, 1)
        s2_batch = np.array([_[4] for _ in batch])

        return s_batch, a_batch, r_batch, t_batch, s2_batch

    def clear(self):
        """Remove all experiences from the buffer."""
        self.buffer.clear()
        self.count = 0

# Example usage:
if __name__ == "__main__":
    buffer = ReplayBuffer(buffer_size=1000)
    buffer.add(np.zeros(5), np.ones(2), 1.0, 0, np.zeros(5))
    print("Buffer size:", buffer.size())
    batch = buffer.sample_batch(1)
    print("Sampled batch shapes:", [arr.shape for arr in batch])
