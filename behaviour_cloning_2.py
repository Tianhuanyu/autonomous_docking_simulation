import os
import torch
torch.set_default_dtype(torch.float64)
import random
import numpy as np
from torch import nn
from torch.utils.data import Dataset
from torch.utils.data import DataLoader

device = "cuda" if torch.cuda.is_available() else "cpu"

class BCDataset(Dataset):

    datadir = 'data/bc'

    def __init__(self):

        self._state = []
        self._action = []

        for filename in os.listdir(self.datadir):

            # Load raw data
            path = os.path.join(self.datadir, filename)
            data = torch.from_numpy(np.loadtxt(path, delimiter=',')).to(device)
            t, f, q, p, r = torch.split(data, [1, 6, 7, 3, 4], 1)

            # Collect state-action pairs
            N = t.shape[0]
            for i in range(N-1):

                # State
                qcurr = q[i, :]
                pcurr = p[i, :]
                rcurr = r[i, :]
                fcurr = f[i, :]
                state = torch.cat((qcurr, pcurr, rcurr, fcurr))
                self._state.append(torch.flatten(state))

                # Action
                qnext = q[i+1, :]
                dt = t[i+1] - t[i]
                dq = (qnext - qcurr)/dt
                self._action.append(torch.flatten(dq))

        # Shuffle data
        self._indices = list(range(len(self)))
        random.shuffle(self._indices)

    def __len__(self):
        return len(self._state)

    def __getitem__(self, idx):
        i = self._indices[idx]
        return self._state[i], self._action[i]

class BCNeuralNetwork(nn.Module):

    def __init__(self):
        super().__init__()
        self.layers = nn.Sequential(
            nn.Linear(20, 500),
            nn.ReLU(),
            nn.Linear(500, 1000),
            nn.ReLU(),
            nn.Linear(1000, 1000),
            nn.ReLU(),
            nn.Linear(1000, 1000),
            nn.ReLU(),
            nn.Linear(1000, 1000),
            nn.ReLU(),
            nn.Linear(1000, 500),
            nn.ReLU(),
            nn.Linear(500, 7),
        )

    def forward(self, x):
        return self.layers(x)

def train_model(filename = 'model_2.nn'):

    # Specify hyper-parameters
    batch_size = 1000
    learning_rate = 0.001

    # Load data
    dataset = BCDataset()
    print("Loaded dataset with", len(dataset), "data points.")
    dataloader = DataLoader(dataset, batch_size=batch_size)

    # Specify model
    model = BCNeuralNetwork().to(device)

    # Setup optimization
    loss_fn = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    # Train model
    try:
        for X_train, y_train in dataloader:
            optimizer.zero_grad()
            output = model(X_train)
            loss = loss_fn(output, y_train)
            loss.backward()
            optimizer.step()
            print("Loss:", loss.item())
    except KeyboardInterrupt:
        print("User quit during training.")
        print("Save model? [Yn]")
        usrin = input('>>')
        if usrin.lower() == 'n':
            return


    # Save model
    torch.save(model.state_dict(), 'nn/'+filename)

def load_model(filename='model_2.nn'):
    model = BCNeuralNetwork()
    model.load_state_dict(torch.load('nn/' + filename))
    model.eval()
    return model

def main():
    train_model()

if __name__ == '__main__':
    main()
