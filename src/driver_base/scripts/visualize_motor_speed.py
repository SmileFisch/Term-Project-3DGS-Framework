import matplotlib.pyplot as plt
import pandas as pd


def visualize_motor_speed_log(file_path):
    """
    Visualize target speed, current speed, and error from the motor_speed_log.txt file.
    :param file_path: Path to the log file
    """
    try:
        # Read the log file
        data = pd.read_csv(
            file_path, header=None, names=["Timestamp", "Target Speed", "Current Speed", "Error"]
        )

        # Convert timestamps to relative time (using the first timestamp as the
        # base)
        data["Timestamp"] = data["Timestamp"] - data["Timestamp"][0]

        # Create the plot
        plt.figure(figsize=(12, 6))

        # Plot target speed and current speed
        plt.plot(data["Timestamp"], data["Target Speed"], label="Target Speed", linewidth=2)
        plt.plot(data["Timestamp"], data["Current Speed"], label="Current Speed", linewidth=2)

        # Plot the error
        plt.plot(data["Timestamp"], data["Error"], label="Error", linestyle="--", linewidth=1)

        # Chart settings
        plt.title("Motor Speed Control Visualization")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed and Error (m/s)")
        plt.legend()
        plt.grid(True)

        # Display the chart
        plt.tight_layout()
        plt.show()

    except FileNotFoundError:
        print(f"Error: Log file '{file_path}' not found.")
    except pd.errors.EmptyDataError:
        print("Error: Log file is empty or corrupted.")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


# Call the function
if __name__ == "__main__":
    log_file_path = "~/ros2_sa/motor_speed_log.txt"  # Path to the log file
    visualize_motor_speed_log(log_file_path)
