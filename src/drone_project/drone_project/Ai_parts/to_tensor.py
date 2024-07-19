import torch
import pandas as pd

def csv_to_tensor(file_path):
    # Read the CSV file into a Pandas DataFrame
    df = pd.read_csv(file_path)
    
    # Convert the DataFrame to a NumPy array
    np_array = df.values
    
    # Convert the NumPy array to a PyTorch tensor
    tensor = torch.tensor(np_array, dtype=torch.float32)
    
    return tensor

def save_tensor(tensor, save_path):
    torch.save(tensor, save_path)
    print(f"Tensor saved to {save_path}")

# Usage example
csv_file_path = '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/log_flight/flight_data1.csv'
tensor_save_path = '/home/naor/Desktop/naor/study/Ai_gps_ros2/src/drone_project/drone_project/Ai_parts/saved_tensor/flight_tensor.pt'

# Convert CSV to tensor
tensor = csv_to_tensor(csv_file_path)

# Save tensor to file
save_tensor(tensor, tensor_save_path)
