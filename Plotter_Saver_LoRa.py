import csv
import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import datetime

# --------------- CONFIGURATION ---------------
SERIAL_PORT = '/dev/ttyUSB0'  # Cambia esto según tu puerto serie (ejemplo: '/dev/ttyUSB0' en Linux)
BAUD_RATE = 115200      # Ajusta la velocidad de transmisión si es necesario
CSV_FILENAME = 'data_log.csv'
MAX_POINTS = 100      # Máximo de puntos a mostrar en la gráfica
# ---------------------------------------------

# Inicializa el archivo CSV con encabezados si está vacío
with open(CSV_FILENAME, mode='a', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    if csvfile.tell() == 0:  # Si el archivo está vacío, escribir encabezados
        csv_writer.writerow(['Unix Epoch', 'Latitude', 'Longitude', 'Altitude (mm)',
                             'Accel X', 'Accel Y', 'Accel Z',
                             'Gyro X', 'Gyro Y', 'Gyro Z', 'Temp (F)'])

# Global lists for plotting
timestamps, accel_x, accel_y, accel_z = [], [], [], []
gyro_x, gyro_y, gyro_z = [], [], []

# Create a figure with two subplots: one for accelerometer, one for gyroscope
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
ax1.set_title('Accelerometer Data')
ax1.set_xlabel('Time')
ax1.set_ylabel('Acceleration')
ax2.set_title('Gyroscope Data')
ax2.set_xlabel('Time')
ax2.set_ylabel('Rotation')

# Open serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except serial.SerialException:
    print(f"Error: No se pudo abrir el puerto {SERIAL_PORT}.")
    exit()

def update(frame):
    if ser.in_waiting:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except UnicodeDecodeError:
            return

        if line:
            parts = line.split(',')
            if len(parts) != 11:
                return  # Evita procesar líneas incorrectas

            try:
                # Guardar la línea en CSV
                with open(CSV_FILENAME, mode='a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(parts)

                # Convertir valores
                timestamp = datetime.datetime.fromtimestamp(float(parts[0]))
                ax_val, ay_val, az_val = map(float, parts[4:7])
                gx_val, gy_val, gz_val = map(float, parts[7:10])

                # Agregar datos a listas
                timestamps.append(timestamp)
                accel_x.append(ax_val)
                accel_y.append(ay_val)
                accel_z.append(az_val)
                gyro_x.append(gx_val)
                gyro_y.append(gy_val)
                gyro_z.append(gz_val)

                # Limitar tamaño de listas
                if len(timestamps) > MAX_POINTS:
                    timestamps.pop(0)
                    accel_x.pop(0)
                    accel_y.pop(0)
                    accel_z.pop(0)
                    gyro_x.pop(0)
                    gyro_y.pop(0)
                    gyro_z.pop(0)

            except (ValueError, IndexError):
                return  # Ignora errores en conversión

    # Clear previous plots
    ax1.cla()
    ax2.cla()

    # Reconfigure axes
    ax1.set_title('Accelerometer Data')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Acceleration')
    ax2.set_title('Gyroscope Data')
    ax2.set_xlabel('Time')
    ax2.set_ylabel('Rotation')

    # Plot accelerometer data
    ax1.plot(timestamps, accel_x, label='Accel X', color='r')
    ax1.plot(timestamps, accel_y, label='Accel Y', color='g')
    ax1.plot(timestamps, accel_z, label='Accel Z', color='b')
    ax1.legend(loc='upper left')
    ax1.tick_params(axis='x', rotation=45)

    # Plot gyroscope data
    ax2.plot(timestamps, gyro_x, label='Gyro X', color='r')
    ax2.plot(timestamps, gyro_y, label='Gyro Y', color='g')
    ax2.plot(timestamps, gyro_z, label='Gyro Z', color='b')
    ax2.legend(loc='upper left')
    ax2.tick_params(axis='x', rotation=45)

    plt.tight_layout()

# Create animation that updates the plot
ani = FuncAnimation(fig, update, interval=100)

plt.show()
