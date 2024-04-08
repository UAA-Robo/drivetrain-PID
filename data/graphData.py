"""
Install pandas and matplotlip. Run this file in this directory with `python3 graphData.py`
"""

import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file
csv = "PID_P0.5_I0.0_D0.0.csv"
df = pd.read_csv(csv)

# plotting a line graph 

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
