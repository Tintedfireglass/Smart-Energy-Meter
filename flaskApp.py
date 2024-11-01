from flask import Flask, jsonify, request
import requests
import csv
import os

app = Flask(__name__)

# Replace with your Pico's IP address
PICO_IP = 'http://192.168.1.100'  # Update this to your Pico's IP
CSV_FILE = 'readings.csv'

# Function to write readings to CSV
def save_reading_to_csv(data):
    file_exists = os.path.isfile(CSV_FILE)
    
    with open(CSV_FILE, mode='a', newline='') as csvfile:
        fieldnames = ['timestamp', 'voltage', 'current', 'power', 'energy', 'relayState']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        # Write header only if file did not exist
        if not file_exists:
            writer.writeheader()

        # Prepare data for CSV
        writer.writerow(data)

@app.route('/relay', methods=['POST'])
def control_relay():
    state = request.json.get('state')
    if state not in ['on', 'off']:
        return jsonify({'error': 'Invalid state. Use "on" or "off".'}), 400

    # Send request to Pico to change relay state
    response = requests.get(f"{PICO_IP}/relay?state={state}")
    
    if response.ok:
        return jsonify(response.json())
    else:
        return jsonify({'error': 'Failed to control relay.'}), 500

@app.route('/readings', methods=['GET'])
def get_readings():
    # Send request to Pico to get readings
    response = requests.get(f"{PICO_IP}/readings")

    if response.ok:
        data = response.json()
        
        # Create a combined timestamp string
        timestamp = f"{data['timestamp']['hour']:02}:{data['timestamp']['minute']:02}:{data['timestamp']['second']:02} " \
                    f"{data['timestamp']['day']:02}/{data['timestamp']['month']:02}/{data['timestamp']['year']}"
        
        # Add the formatted timestamp to the data dictionary
        data['timestamp'] = timestamp

        save_reading_to_csv(data)  # Save the readings to CSV
        return jsonify(data)
    else:
        return jsonify({'error': 'Failed to retrieve readings.'}), 500

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
