import os
import requests
import time
import socket
import json

# Configuration
CSV_FILE_PATH = "/home/hemant/grain_summary.csv"   # Path to your CSV
SERVER_URL = "https://retra-ai-w7wm.onrender.com/api/upload-csv"
RESPONSE_FILE = "/home/hemant/response.json"
CHECK_INTERVAL = 5  # seconds

def is_connected(host="8.8.8.8", port=53, timeout=3):
    """Check if Raspberry Pi has network access."""
    try:
        socket.setdefaulttimeout(timeout)
        socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect((host, port))
        return True
    except Exception:
        return False

def upload_csv(file_path, url, response_path):
    """Upload CSV file and save server response."""
    if not os.path.exists(file_path):
        print(f"CSV file {file_path} not found.")
        return False

    with open(file_path, 'rb') as f:
        files = {'file': (os.path.basename(file_path), f, 'text/csv')}
        try:
            response = requests.post(url, files=files)
            # Save response to JSON file
            with open(response_path, 'w') as resp_file:
                json.dump(response.json(), resp_file, indent=4)
            
            if response.status_code == 200:
                print("Upload successful. Deleting CSV...")
                os.remove(file_path)
                return True
            else:
                print(f"Upload failed. Status code: {response.status_code}, Response: {response.text}")
                return False
        except requests.exceptions.RequestException as e:
            print(f"Error uploading CSV: {e}")
            return False
        except json.JSONDecodeError:
            print(f"Server response is not JSON: {response.text}")
            return False

def main():
    while True:
        if is_connected():
            print("Network detected. Attempting to upload CSV...")
            upload_csv(CSV_FILE_PATH, SERVER_URL, RESPONSE_FILE)
        else:
            print("No network connection. Waiting...")
        time.sleep(CHECK_INTERVAL)

if __name__ == "__main__":
    main()
