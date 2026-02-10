import csv
import numpy as np 
import datetime

import serial
import time

def read_from_serial(port, baudrate, output_file, num_of_iterations):
    # Open the serial port
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"Connected to {port} at {baudrate} baudrate.")
    except Exception as e:
        print(f"Failed to connect to {port}: {e}")
        return
    
    # Open the output file
    with open(output_file, 'w') as f:
        try:
            current_line_count = 0
            while num_of_iterations > current_line_count:
                # Read a line from the serial port
                data = ser.readline().decode('utf-8').strip()

                if data:
                    # Write the data to the file with timestamp                    
                    f.write(data + '\n')
                    current_line_count += 1

                
                # Optional: Limit the read loop or set a break condition
                # time.sleep(0.1)  # Uncomment if you want to slow down the reading loop

        except KeyboardInterrupt:
            print("Stopping the serial reading...")

        finally:
            # Close the serial port
            ser.close()
            print("Serial port closed.")



import numpy as np
import xlsxwriter

def extract_amplitudes(input_file_path, output_xlsx_path, max_sweeps_in_chart=5):
    with open(input_file_path, 'r') as infile:
        workbook = xlsxwriter.Workbook(output_xlsx_path)
        worksheet = workbook.add_worksheet("Sweep Data")
        chart = workbook.add_chart({'type': 'line'})

        # Generate and round x-axis header values
        range_values = np.arange(0.050, 1.05, 0.0025) / 1.48
        range_values = [float(f"{v:.4g}") for v in range_values]

        # Write header to the worksheet
        for col, val in enumerate(range_values):
            worksheet.write(0, col, val)
        worksheet.write(0, len(range_values), "Temp")
        worksheet.write(0, len(range_values) + 1, "Divisor")

        sweep_row = 1
        amplitudes = []
        temp = ""
        divisor = ""
        sweep_index = 0

        for line in infile:
            line = line.strip()

            # Start of new sweep
            if "Full sweep received. Printing all amplitudes and indexes:" in line:
                if len(amplitudes) == 400:
                    for col, amp in enumerate(amplitudes):
                        worksheet.write(sweep_row, col, float(amp))
                    worksheet.write(sweep_row, 400, temp)
                    worksheet.write(sweep_row, 401, divisor)

                    # Add up to max_sweeps_in_chart for charting
                    if sweep_index < max_sweeps_in_chart:
                        chart.add_series({
                            'name': f"Sweep {sweep_row}",
                            'categories': ['Sweep Data', 0, 0, 0, 399],
                            'values':     ['Sweep Data', sweep_row, 0, sweep_row, 399],
                            'line': {'width': 1.0},
                        })
                        sweep_index += 1

                    sweep_row += 1
                amplitudes = []

            # Parse temperature and divisor
            elif "Temp" in line:
                try:
                    temp_string = line.split("<>")
                    strtemp = temp_string[0].split(":")
                    strdivisor = temp_string[1].split(":")
                    temp = float(strtemp[1])
                    divisor = float(strdivisor[1])
                except:
                    temp, divisor = "", ""

            # Parse amplitudes
            elif "Amplitude" in line and "Index" in line:
                try:
                    amplitude = line.split("Amplitude ")[-1]
                    amplitudes.append(float(amplitude))
                except ValueError:
                    amplitudes.append(0.0)

        # Insert chart near the top
        worksheet.insert_chart("D2", chart)
        workbook.close()

if __name__ == "__main__":
    import os
    # Replace th
    # ese with your own values
    port = "COM4"  # e.g., "COM3" for Windows, "/dev/ttyUSB0" for Linux
    baudrate = 250000
    # Get today's date string
    today_str = datetime.datetime.now().strftime("%Y-%m-%d")
    dt_string = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # Create the directory if it doesn't exist
    output_dir = os.path.join("data", today_str)
    os.makedirs(output_dir, exist_ok=True)
    serial_data_filepath = os.path.join(output_dir, f"{dt_string}_output_data.txt")
    OUTPUT_DATAFILE = os.path.join(output_dir, f"{dt_string}_sweep_amplitudes.xlsx")
    print(f"Reading data from serial port {port} and saving to {serial_data_filepath}...")

    # Call the function to read from serial and write to file
    try:
        read_from_serial(port, baudrate, serial_data_filepath, num_of_iterations=2000)
    except KeyboardInterrupt:
        print("DONE Reading from serial")


    # Path to your input file
    OUTPUT_DATAFILE = f'data\\{dt_string}_sweep_amplitudes.xlsx'  # Path to your output CSV

    extract_amplitudes(serial_data_filepath, OUTPUT_DATAFILE)
    
    print(f"Amplitudes from each sweep have been written to {OUTPUT_DATAFILE}.")
