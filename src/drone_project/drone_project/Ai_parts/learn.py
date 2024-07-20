import torch
import torch.nn as nn
import torch.optim as optim
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Check if GPU is available
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
print(f'Using device: {device}')

# Load the tensor
tensor_path = '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/Ai_parts/saved_tensor/flight_tensor.pt'
data = torch.load(tensor_path)

# Handle NaN values and infinite values
data_np = data.numpy()
col_means = np.nanmean(data_np, axis=0)

# Replace NaNs with column means
inds = np.where(np.isnan(data_np))
data_np[inds] = np.take(col_means, inds[1])

# Replace infinite values with column means
data_np[np.isinf(data_np)] = np.take(col_means, np.isinf(data_np).nonzero()[1])

# If column means are still NaN or infinite, replace them with 0
col_means[np.isnan(col_means)] = 0
col_means[np.isinf(col_means)] = 0
data_np = np.nan_to_num(data_np, nan=0, posinf=0, neginf=0)
data = torch.tensor(data_np, dtype=torch.float32)

# Extract features and labels
X = data[:, :-3]  # All columns except the last three (non-GPS features)
y = data[:, -3:]  # The last three columns (GPS labels)

# Normalize the features
scaler_X = StandardScaler()
X = scaler_X.fit_transform(X)
X = torch.tensor(X, dtype=torch.float32)

# Verify the data does not contain NaNs or infinite values after preprocessing
assert not torch.isnan(X).any(), "X contains NaNs"
assert not torch.isinf(X).any(), "X contains infinite values"
assert not torch.isnan(y).any(), "y contains NaNs"
assert not torch.isinf(y).any(), "y contains infinite values"

# Split the data into training and testing sets (80% train, 20% test) without shuffling
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, shuffle=False)

# Move tensors to the GPU
X_train = X_train.clone().detach().to(device)
X_test = X_test.clone().detach().to(device)
y_train = y_train.clone().detach().to(device)
y_test = y_test.clone().detach().to(device)


# Define the neural network model
class GPSPredictor(nn.Module):
    def __init__(self):
        super(GPSPredictor, self).__init__()
        self.fc1 = nn.Linear(X_train.shape[1], 256)
        self.dropout1 = nn.Dropout(0.3)
        self.fc2 = nn.Linear(256, 128)
        self.dropout2 = nn.Dropout(0.3)
        self.fc3 = nn.Linear(128, 64)
        self.dropout3 = nn.Dropout(0.3)
        self.fc4 = nn.Linear(64, 32)
        self.dropout4 = nn.Dropout(0.3)
        self.fc5 = nn.Linear(32, 3)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.dropout1(x)
        x = torch.relu(self.fc2(x))
        x = self.dropout2(x)
        x = torch.relu(self.fc3(x))
        x = self.dropout3(x)
        x = torch.relu(self.fc4(x))
        x = self.dropout4(x)
        x = self.fc5(x)
        return x


# Initialize the model, loss function, and optimizer
model = GPSPredictor().to(device)
criterion = nn.SmoothL1Loss()  # Huber Loss
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Train the model
num_epochs = 100000
for epoch in range(num_epochs):
    model.train()
    optimizer.zero_grad()
    outputs = model(X_train)
    loss = criterion(outputs, y_train)
    loss.backward()
    optimizer.step()

    if (epoch + 1) % 10 == 0:
        print(f'Epoch [{epoch + 1}/{num_epochs}], Loss: {loss.item():.4f}')

# Evaluate the model
model.eval()
with torch.no_grad():
    predictions = model(X_test)
    test_loss = criterion(predictions, y_test)
    print(f'Test Loss: {test_loss.item():.4f}')

# Calculate the percentage of correct predictions
threshold = 0.1  # Define a threshold for considering a prediction as "correct"
correct_predictions = torch.sum(torch.abs(predictions - y_test) < threshold).item()
total_predictions = y_test.numel()  # Total number of elements in y_test
accuracy = (correct_predictions / total_predictions) * 100

print(f'Percentage of correct predictions: {accuracy:.2f}%')

# Save the model if needed
torch.save(model.state_dict(),
           '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/Ai_parts/saved_tensor/gps_predictor_model.pth')

print("Model training and evaluation complete.")

# Move predictions and y_test back to CPU for plotting
predictions = predictions.cpu().numpy()
y_test = y_test.cpu().numpy()

# Define y_result as the predicted positions
y_result = predictions

# 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot training locations (moving data back to CPU for plotting)
y_train = y_train.cpu().numpy()
ax.scatter(y_train[:, 0], y_train[:, 1], y_train[:, 2], c='g', marker='^', label='Train')

# Plot actual test locations
ax.scatter(y_test[:, 0], y_test[:, 1], y_test[:, 2], c='b', marker='o', label='Actual Test')

# Plot predicted locations for the test set (y_result)
ax.scatter(y_result[:, 0], y_result[:, 1], y_result[:, 2], c='r', marker='x', label='Predicted')

ax.set_xlabel('X GPS')
ax.set_ylabel('Y GPS')
ax.set_zlabel('Z GPS')
ax.legend()

plt.show()
