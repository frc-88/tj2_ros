import os
import sys
import pandas
import numpy as np
from matplotlib import pyplot as plt


def extract_row(buf, idx):
    """Extract row from 2D array.

    Keyword arguments:
    buf -- matrix containing plot data
    idx -- index of desired plot in buf

    Returns:
    Desired list of data from buf
    """
    return np.squeeze(buf[idx : idx + 1, :])


def plot_time_responses(state_labels, u_labels, states, inputs, t, x_rec, ref_rec, u_rec, title=None):
    """Plots time-domain responses of the system and the control inputs.

    Keyword arguments:
    time -- list of timesteps corresponding to references
    x_rec -- recording of state estimates from generate_time_responses()
    ref_rec -- recording of references from generate_time_responses()
    u_rec -- recording of inputs from generate_time_responses()
    title -- title for time-domain plots (default: "Time-domain responses")
    """
    plt.figure()
    subplot_max = states + inputs
    for i in range(states):
        plt.subplot(subplot_max, 1, i + 1)
        if states + inputs > 3:
            plt.ylabel(
                state_labels[i],
                horizontalalignment="right",
                verticalalignment="center",
                rotation=45,
            )
        else:
            plt.ylabel(state_labels[i])
        if i == 0:
            if title is None:
                plt.title("Time-domain responses")
            else:
                plt.title(title)
        plt.plot(t, extract_row(x_rec, i), label="Estimated state")
        plt.plot(t, extract_row(ref_rec, i), label="Reference")
        plt.legend()

    for i in range(inputs):
        plt.subplot(subplot_max, 1, states + i + 1)
        if states + inputs > 3:
            plt.ylabel(
                u_labels[i],
                horizontalalignment="right",
                verticalalignment="center",
                rotation=45,
            )
        else:
            plt.ylabel(u_labels[i])
        plt.plot(t, extract_row(u_rec, i), label="Control effort")
        plt.legend()
    plt.xlabel("Time (s)")

def main():
    if len(sys.argv) == 1:
        directory = "data"
        filenames = os.listdir(directory)
        filenames = [name for name in filenames if name.endswith(".csv")]
        paths = [os.path.join(directory, f) for f in filenames] # add path to each file
        paths.sort(key=lambda x: os.path.getmtime(x))

        path = paths[-1]
    else:
        path = sys.argv[1]
    print("Using path: %s" % path)

    df = pandas.read_csv(path)

    print(df.columns)

    timestamps = df["timestamp"]
    
    state_labels = [("Azimuth", "rad"), ("Azimuth velocity", "rad/s"), ("Wheel velocity", "rad/s")]
    u_labels = [("Lo Voltage", "V"), ("Hi Voltage", "V")]

    x_rec = np.array([
        df["x_hat azimuth"],
        df["x_hat azimuth velocity"],
        df["x_hat wheel velocity"],
    ])
    # ref_rec = np.array([
    #     df["reference azimuth"],
    #     df["reference azimuth velocity"],
    #     df["reference wheel velocity"],
    # ])
    ref_rec = np.array([
        df["next reference azimuth"],
        df["next reference azimuth velocity"],
        df["next reference wheel velocity"],
    ])
    u_rec = np.array([
        df["lo next voltage"],
        df["hi next voltage"],
    ])

    plot_time_responses(state_labels, u_labels, len(state_labels), len(u_labels), timestamps, x_rec, ref_rec, u_rec)
    plt.gcf().set_size_inches(100, 50)

    # plt.figure()
    # plt.subplot(2, 1, 1)
    # plt.plot(timestamps, df["module speed setpoint"])

    # plt.subplot(2, 1, 2)
    # plt.plot(timestamps, df["module angle setpoint"])

    plt.show()


main()
