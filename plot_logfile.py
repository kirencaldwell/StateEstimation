import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import sys

plot_attitude = False 
plot_gyro_bias = False 
plot_accel_bias = False
plot_w = False
plot_measurements = False 
plot_accel_3d = False 
if len(sys.argv) == 1:
    plot_attitude = True
    plot_gyro_bias = True
    plot_accel_bias = True
    plot_w = True
    plot_measurements = True
    plot_accel_3d = True
else:
    for i in range(1, len(sys.argv)):
        arg = sys.argv[i]
        if arg == "attitude":
            plot_attitude = True
        if arg == "gyro_bias":
            plot_gyro_bias = True
        if arg == "accel_bias":
            plot_accel_bias = True
        if arg == "ang_vel":
            plot_w = True
        if arg == "measurements":
            plot_measurements = True
        if arg == "accel_3d":
            plot_accel_3d = True
    

mpl.rcParams['figure.figsize'] = (16.0, 8.0)
mpl.rcParams['grid.color'] = (0.9, 0.9, 0.9)
mpl.rcParams['grid.linestyle'] = '-'
mpl.rcParams['grid.linewidth'] = 0.5
mpl.rcParams['lines.markersize'] = 5
mpl.rcParams['figure.dpi'] = 100
mpl.rcParams['font.size'] = 8
mpl.rcParams['legend.fontsize'] = 'small'
mpl.rcParams['figure.titlesize'] = 'small'

df = pd.read_csv("logfile.csv")
df.rename(columns=df.iloc[0]).drop(df.index[0])

# plot attitude
if plot_attitude:
    fig, axs = plt.subplots(3, 3)
    for i in range(0, 3):
        for j in range(0, 3):
            true_name = "R_bn" + str(i) + str(j)
            est_name = "R_bn_hat" + str(i) + str(j)
            Rij_true = df.filter(regex=true_name)
            Rij_est = df.filter(regex=est_name)
            axs[i,j].plot(df.t, Rij_true, linewidth=4)
            axs[i,j].plot(df.t, Rij_est, linewidth=1)
            axs[i,j].set_ylim([-1.1, 1.1])
            axs[i,j].grid(True)
    axs[i,j].legend(["true", "estimated"])
    plt.show()

# plot gyro bias
if plot_gyro_bias:
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        col_name = "b_g" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        col_name = "b_g_hat" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        axs[i].grid(True)
        axs[i].set_ylabel("b_g" + str(i))
    axs[i].legend(["true", "estimated"])
    plt.show()

# plot accel bias
if plot_accel_bias:
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        col_name = "b_a" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        col_name = "b_a_hat" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        axs[i].grid(True)
        axs[i].set_ylabel("b_a" + str(i))
    axs[i].legend(["true", "estimated"])
    plt.show()

# plot estimated angular velocity
if plot_w:
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        col_name = "gyro" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        col_name = "w_bn_hat" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        col_name = "w_bn" + str(i)
        y_df = df.filter(regex=col_name)
        axs[i].plot(df.t, y_df, linewidth=4)
        axs[i].grid(True)
        axs[i].set_ylabel("w" + str(i))
    axs[i].legend(["measured", "estimated", "true"])
    plt.show()

# plot measurements
if plot_measurements:
    y_names = ["accel", "magn", "gyro"]
    fig, axs = plt.subplots(3)
    for i in range(0, 3):
        for j in range(0, 3):
            col_name = y_names[i] + str(j)
            y_df = df.filter(regex=col_name)
            axs[i].plot(df.t, y_df, linewidth=4)
            axs[i].grid(True)
            axs[i].set_ylabel(y_names[i])
    plt.show()

# plot accelerometer 3d data
if plot_accel_3d:
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(df.accel0, df.accel1, df.accel2, marker='o')
    ax.grid(True)
    plt.show()