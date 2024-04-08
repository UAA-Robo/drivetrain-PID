"""
Loops through every csv in the pidCSV folder and graphs them.
Install pandas and matplotlip. Run this file in this directory with `python3 graphData.py`
"""

import pandas as pd
import matplotlib.pyplot as plt
import os


folder_path = 'pidCSV'

# Iterate over each entry in the folder path and plot each csv
for i, csv in enumerate(os.listdir(folder_path)):

    df = pd.read_csv(f"{folder_path}/{csv}")

    plt.figure(i)
    plt.title(csv)

    plt.subplot(1, 2, 1)
    plt.plot(df["msecElapsed"], df["Control"]) 
    plt.xlabel('Time Elapsed (msec)')
    plt.ylabel('Control Value: Drivetrain Voltage (V)')

    plt.subplot(1, 2, 2)
    plt.plot(df["msecElapsed"], df["Process"]) 
    plt.plot(df["msecElapsed"], df["Setpoint"], label="Setpoint",  linestyle='--') 
    plt.xlabel('Time Elapsed (msec)')
    plt.ylabel('Process Value: Distance to Position (inch)')
    plt.legend()
    
plt.show() 
