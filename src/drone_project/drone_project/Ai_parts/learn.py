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
tensor_path = '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/Ai_parts/saved_tensor/flight_tensor1.pt'
data = torch.load(tensor_path)

# Check for NaN values and replace them with the mean of the column
data = torch.tensor(np.nan_to_num(data.numpy(), nan=np.nanmean(data.numpy(), axis=0)))

# Extract features and labels
X = data[:, :-3]  # All columns except the last three (non-GPS features)
y = data[:, -3:]  # The last three columns (GPS labels)

# Normalize the features and labels
scaler_X = StandardScaler()
scaler_y = StandardScaler()
X = scaler_X.fit_transform(X)
y = scaler_y.fit_transform(y)

# Split the data into training and testing sets (80% train, 20% test) without shuffling
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, shuffle=False)

# Convert the numpy arrays back to PyTorch tensors and move them to the GPU
X_train = torch.tensor(X_train, dtype=torch.float32).to(device)
X_test = torch.tensor(X_test, dtype=torch.float32).to(device)
y_train = torch.tensor(y_train, dtype=torch.float32).to(device)
y_test = torch.tensor(y_test, dtype=torch.float32).to(device)

# Define the neural network model
class GPSPredictor(nn.Module):
    def __init__(self):
        super(GPSPredictor, self).__init__()
        self.fc1 = nn.Linear(X_train.shape[1], 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, 16)
        self.fc4 = nn.Linear(16, 3)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        x = self.fc4(x)
        return x

# Initialize the model, loss function, and optimizer
model = GPSPredictor().to(device)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Train the model
num_epochs = 100
for epoch in range(num_epochs):
    model.train()
    optimizer.zero_grad()
    outputs = model(X_train)
    loss = criterion(outputs, y_train)
    loss.backward()
    optimizer.step()
    
    if (epoch+1) % 10 == 0:
        print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')

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
torch.save(model.state_dict(), '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/Ai_parts/saved_tensor/gps_predictor_model.pth')

print("Model training and evaluation complete.")

# Move predictions and y_test back to CPU for plotting
predictions = predictions.cpu().numpy()
y_test = y_test.cpu().numpy()

# 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot training locations (moving data back to CPU for plotting)
y_train = y_train.cpu().numpy()
ax.scatter(y_train[:, 0], y_train[:, 1], y_train[:, 2], c='g', marker='^', label='Train')

# Plot predicted locations for the test set
ax.scatter(predictions[:, 0], predictions[:, 1], predictions[:, 2], c='r', marker='x', label='Predicted')

ax.set_xlabel('X GPS')
ax.set_ylabel('Y GPS')
ax.set_zlabel('Z GPS')
ax.legend()

plt.show()
