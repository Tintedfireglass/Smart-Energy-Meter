from flask import Flask, jsonify, request, render_template
import csv
import os
import serial
import time
import json
import random

app = Flask(__name__)

SERIAL_PORT = 'COM6'
BAUD_RATE = 115200
CSV_FILE = 'static/readings.csv'

# Store the last reading to use as reference for dummy values
last_reading = {
    "voltage": 230.0,
    "current": 3.50,
    "power": 805,
    "energy": 805,    
    "relayState": "on"
}

# Global serial object
ser = None

def init_serial():
    """Initialize serial connection with error handling"""
    global ser
    try:
        if ser is not None:
            ser.close()
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Serial Connected!!!")
        time.sleep(2)
        return True
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {SERIAL_PORT}. {e}")
        ser = None
        return False

# Initial serial connection attempt
init_serial()

def save_reading_to_csv(data):
    os.makedirs('static', exist_ok=True)
    file_exists = os.path.isfile(CSV_FILE)
    
    with open(CSV_FILE, mode='a', newline='') as csvfile:
        fieldnames = ['timestamp', 'voltage', 'current', 'power', 'energy', 'relayState']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
            writer.writeheader()
        writer.writerow(data)

def generate_dummy_reading(last_reading):
    """Generate dummy readings based on last known values"""
    try:
        # Generate values with small random variations from the last reading
        voltage = round(last_reading["voltage"] + random.uniform(-2, 2), 1)
        current = round(last_reading["current"] + random.uniform(-0.05, 0.05), 2)
        
        # Keep values within realistic ranges
        voltage = max(min(voltage, 240), 220)
        current = max(min(current, 4), 0)
        
        power = round(voltage * current, 2)
        energy = power
        
        return {
            "voltage": voltage,
            "current": current,
            "power": power,
            "energy": energy,
            "relayState": last_reading["relayState"]
        }
    except Exception as e:
        print(f"Error generating dummy reading: {e}")
        # Return default values if there's an error
        return {
            "voltage": 230.0,
            "current": 3.50,
            "power": 805,
            "energy": 805,    
            "relayState": "on"
        }

def read_serial_data():
    """Read data from serial port with error handling"""
    global ser
    
    if ser is None or not ser.is_open:
        if not init_serial():
            return None

    try:
        response = ser.readline().decode().strip()
        if response:
            return response
    except (serial.SerialException, UnicodeDecodeError) as e:
        print(f"Serial read error: {e}")
        ser = None  # Reset serial connection
        return None
    
    return None

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/readings', methods=['GET'])
def get_readings():
    global last_reading
    timestamp = time.strftime("%H:%M:%S %d/%m/%Y")
    
    # Try to get serial data
    serial_data = read_serial_data()
    
    if serial_data:
        try:
            data = json.loads(serial_data)
            data['timestamp'] = timestamp
            last_reading = {k: v for k, v in data.items() if k != 'timestamp'}
            save_reading_to_csv(data)
            return jsonify(data)
        except json.JSONDecodeError as e:
            print(f"Failed to parse serial data: {e}")
    
    # Generate and save dummy data if no serial data available
    dummy_data = generate_dummy_reading(last_reading)
    dummy_data['timestamp'] = timestamp
    save_reading_to_csv(dummy_data)
    return jsonify(dummy_data)

@app.route('/relay', methods=['POST'])
def control_relay():
    global last_reading, ser
    state = request.json.get('state', 'off')
    
    # Update relay state in last_reading
    last_reading['relayState'] = state
    
    # Try to send command to serial if connected
    if ser and ser.is_open:
        try:
            ser.write(f"RELAY_{state.upper()}\n".encode())
        except serial.SerialException as e:
            print(f"Failed to send relay command: {e}")
            init_serial()  # Try to reinitialize serial connection
    
    return jsonify({'status': 'success', 'relayState': state})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)