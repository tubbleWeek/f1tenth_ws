import pandas as pd

def save_trajectories_to_csv(trajectories, filename='all_trajectories.csv'):
    """
    Save trajectories to a CSV file.

    Parameters:
    - trajectories: List of dictionaries containing trajectory data.
                    Each dictionary should have keys: 'env_index', 'goal_index', 
                    'trajectory_index', 'step', 'x', 'y', 'theta', 'velocity', 'steering_angle'.
    - filename: Name of the CSV file to save the trajectories.
    """
    # Create an empty DataFrame to store all trajectories
    all_trajectories_df = pd.DataFrame(columns=[
        'env_index', 'goal_index', 'trajectory_index', 'step',
        'x', 'y', 'theta', 'velocity', 'steering_angle'
    ])

    for trajectory in trajectories:
        # Append each trajectory to the DataFrame
        row = {
            'env_index': trajectory['env_index'],
            'goal_index': trajectory['goal_index'],
            'trajectory_index': trajectory['trajectory_index'],
            'step': trajectory['step'],
            'x': trajectory['x'],
            'y': trajectory['y'],
            'theta': trajectory['theta'],
            'velocity': trajectory['velocity'],
            'steering_angle': trajectory['steering_angle']
        }
        all_trajectories_df = all_trajectories_df.append(row, ignore_index=True)

    # Save all trajectories to a single CSV file
    all_trajectories_df.to_csv(filename, index=False)
    print(f"All trajectories saved to {filename}")