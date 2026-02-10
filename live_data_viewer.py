import numpy as np 
import datetime

import serial
import time
from matplotlib.animation import FuncAnimation
import threading
from queue import Queue

import matplotlib.pyplot as plt


def read_and_plot_serial(port, baudrate, rf_factor):
    def read_serial_data(queue, port, baudrate):
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port} at {baudrate} baudrate.")
        except Exception as e:
            print(f"Failed to connect to {port}: {e}")
            return

        sweep_id = None
        amplitudes = []
        in_sweep = False

        temp = 0
        divisor = 0

        try:
            while True:
                line = ser.readline().decode('utf-8').strip()

                if "Full sweep received" in line:
                    if len(amplitudes) == 400:
                        queue.put((amplitudes, temp, divisor))
                    else:
                        print("ERROR: Incomplete sweep data received.")

                    amplitudes = []  # Reset amplitudes for the new sweep
                    in_sweep = True

                if "Temp" in line:
                    temp_string = line.split("<>")
                    strtemp = temp_string[0].split(":")
                    strdivisor = temp_string[1].split(":")
                    temp = float(strtemp[1])
                    divisor = float(strdivisor[1])

                if "Amplitude" in line and "Index" in line:
                    amplitude = float(line.split("Amplitude ")[-1])
                    amplitudes.append(amplitude)

        except KeyboardInterrupt:
            print("Stopping serial reading...")
        finally:
            ser.close()
            print("Serial port closed.")


    def update_plot(frame):
        while not data_queue.empty():
            data = data_queue.get()
            range_values = np.arange(START, STOP, 0.0025) / rf_factor
            try:
                amplitudes, temp, divisor = data
                line.set_ydata(amplitudes)
                line.set_xdata(range_values)
                #ax.set_title(f"Temp: {temp}, Divisor: {divisor}")
                ax.set_ylim(min(amplitudes) - 100, max(amplitudes) + 1000)
                #ax.set_ylim(0, 80000)   # set y axis manually
                fig.canvas.draw() 
            except ValueError:
                print(f"Invalid data: {data}")

    data_queue = Queue()
    reader_thread = threading.Thread(target=read_serial_data, args=(data_queue, port, baudrate), daemon=True)
    reader_thread.start()

    x_thresholds = [0.0, 0.06, 0.2, .7]
    y_thresholds =  [400000, 2000, 500, 400]

    # To match the config in the a121 PrintDataConfig object multiply .start_point by 0.0025
    NUM_POINTS = 400
    START = 15 * 0.0025
    STOP = START + (NUM_POINTS * 0.0025)
    range_values = np.arange(START, STOP, 0.0025) / rf_factor
    fig, ax = plt.subplots()
    ax.set_ylabel("Amplitud" \
    "e IMU")
    ax.set_xlabel("Meters")
    line, = ax.plot(range_values, np.zeros(400))  # Adjust size to match the sweep data
    ax.plot([num for num in x_thresholds],  y_thresholds)
    ax.set_ylim(0, 2000)  # Adjust limits as needed
    ax.set_xlim(0, .7)

    ani = FuncAnimation(fig, update_plot, interval=100)
    plt.show()

if __name__ == "__main__":
    PORT = "com5"
    BAUD = 250000
    rf_factor = 1.48
    read_and_plot_serial(port=PORT, baudrate=BAUD, rf_factor=rf_factor)