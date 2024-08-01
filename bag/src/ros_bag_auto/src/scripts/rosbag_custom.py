#!/usr/bin/env python2

import rospy
import psutil # CPU, RAM, MEMORY
import subprocess
import datetime 
import pandas as pd 

from std_msgs.msg import Float32


current_bat_vol = 0

# Check memory
def check_memory():
    memory_present = psutil.virtual_memory().percent
    return memory_present
        
# Check CPU
def check_cpu():
    cpu_percent = psutil.cpu_percent()
    rospy.loginfo("CPU using: {}%".format(cpu_percent))
    return cpu_percent

# Check other system info
def check_other_system_info():
    global current_bat_vol
    battery_voltage = current_bat_vol
   
    # RAM
    ram_present = psutil.virtual_memory().percent
    
    # Time
    current_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    rospy.loginfo("Current Time: {}".format(current_time))
    
    return ram_present, battery_voltage, current_time

# Callback function for battery voltage
def bat_vol_callback(data):
    global current_bat_vol
    current_bat_vol = data.data

def main():
    rospy.init_node('rosbag_auto', anonymous=True)
    rate = rospy.Rate(1) # Frequency of 1Hz
    rosbag_process = None
    data = []
    rospy.Subscriber('/bat_vol', Float32, bat_vol_callback)

    while not rospy.is_shutdown():
        memory_present = check_memory()
        
        if memory_present > 80 and rosbag_process is None:
            rospy.loginfo("Memory is more than 80%. Starting to record rosbag...")
            rosbag_process = subprocess.Popen(["rosbag", "record", "--duration=1h", "-a"])
            
        cpu_percent = check_cpu()
        ram_present, current_time, battery_voltage = check_other_system_info()
        data.append([cpu_percent, ram_present, battery_voltage, memory_present, current_time])

        rate.sleep()
    
    df = pd.DataFrame(data, columns=['CPU (%)', 'RAM (%)', 'Battery Voltage (V)', 'Memory (%)', 'Time'])
    # Export the DataFrame to an Excel file
    df.to_excel('rosbag_auto.xlsx', index=False)
    rospy.loginfo("Data has been saved to file rosbag_auto.xlsx")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
