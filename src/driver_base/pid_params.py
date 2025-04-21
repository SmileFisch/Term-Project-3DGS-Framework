import matplotlib.pyplot as plt


def visualize_log(filename="motor_speed_log.txt"):
    timestamps = []
    target_speeds = []
    current_speeds = []

    try:
        with open(filename, "r") as file:
            for line in file:
                parts = line.strip().split(", ")
                if len(parts) < 3:
                    continue
                timestamps.append(float(parts[0]))
                target_speeds.append(float(parts[1].split(": ")[1]))
                current_speeds.append(float(parts[2].split(": ")[1]))

        plt.plot(timestamps, target_speeds, label="Target Speed")
        plt.plot(timestamps, current_speeds, label="Current Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (m/s)")
        plt.legend()
        plt.title("Motor Speed Over Time")
        plt.show()

    except FileNotFoundError:
        print(f"{filename} not found. Make sure the file exists.")


if __name__ == "__main__":
    visualize_log()
