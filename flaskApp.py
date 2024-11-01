from flask import Flask, jsonify
import requests
import csv
import time
from datetime import datetime
import threading
import os
import logging
from logging.handlers import RotatingFileHandler

app = Flask(__name__)

# Configuration
PICO_IP = "192.168.1.100"  # Pico's IP address - Check it properly everytime
PICO_ENDPOINT = f"http://{PICO_IP}/readings"
CSV_FOLDER = "energy_logs"
LOG_FOLDER = "logs"
LOGGING_INTERVAL = 5  # seconds between data points

# Making sure folders exist
os.makedirs(CSV_FOLDER, exist_ok=True)
os.makedirs(LOG_FOLDER, exist_ok=True)

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        RotatingFileHandler(
            f'{LOG_FOLDER}/energy_logger.log',
            maxBytes=1024*1024,  
            backupCount=5
        ),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

class DataLogger:
    def __init__(self):
        self.latest_data = None
        self.csv_file = None
        self.csv_writer = None
        self.lock = threading.Lock()
        self.initialize_csv()

    def initialize_csv(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{CSV_FOLDER}/energy_data_{timestamp}.csv"
        
        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'Timestamp',
                'Voltage (V)',
                'Current (A)',
                'Power (W)',
                'Energy (Wh)',
                'Device Time'
            ])
            logger.info(f"Created new log file: {filename}")
        except Exception as e:
            logger.error(f"Failed to initialize CSV file: {e}")
            raise

    def log_data(self, data):
        try:
            with self.lock:
                self.latest_data = data
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                
                self.csv_writer.writerow([
                    timestamp,
                    data['voltage'],
                    data['current'],
                    data['power'],
                    data['energy'],
                    data['timestamp']
                ])
                self.csv_file.flush()  # Ensure data is written to disk
                
            logger.debug(f"Logged data point: {data}")
        except Exception as e:
            logger.error(f"Failed to log data: {e}")

    def close(self):
        try:
            if self.csv_file:
                self.csv_file.close()
        except Exception as e:
            logger.error(f"Error closing CSV file: {e}")

data_logger = DataLogger()

def fetch_data_from_pico():
    try:
        response = requests.get(PICO_ENDPOINT, timeout=5)
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        logger.error(f"Failed to fetch data: {e}")
        return None

def continuous_logging():
    while True:
        try:
            data = fetch_data_from_pico()
            if data:
                data_logger.log_data(data)
            else:
                logger.warning("No data received from Pico")
                
        except Exception as e:
            logger.error(f"Error in continuous logging: {e}")
            
        time.sleep(LOGGING_INTERVAL)

# Flask routes
@app.route('/status')
def get_status():
    return jsonify({
        'status': 'running',
        'latest_data': data_logger.latest_data,
        'logging_interval': LOGGING_INTERVAL
    })

@app.route('/latest')
def get_latest():
    return jsonify(data_logger.latest_data if data_logger.latest_data else {'error': 'No data available'})

@app.route('/health')
def health_check():
    return jsonify({
        'status': 'healthy',
        'timestamp': datetime.now().isoformat(),
        'pico_connection': bool(data_logger.latest_data)
    })

def start_server():
    # Start the logging thread
    logging_thread = threading.Thread(target=continuous_logging, daemon=True)
    logging_thread.start()
    
    # Start the Flask server
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    try:
        logger.info("Starting Energy Data Logger")
        start_server()
    except KeyboardInterrupt:
        logger.info("Shutting down Energy Data Logger")
        data_logger.close()
    except Exception as e:
        logger.error(f"Unexpected error: {e}")
        data_logger.close()
