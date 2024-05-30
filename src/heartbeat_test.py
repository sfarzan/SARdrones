import csv
import requests
from src.params import Params as params_

def read_swarm(self):
    """
    Reads the swarm configuration file, which includes the list of nodes in the swarm.
    The function supports both online and offline modes.
    In online mode, it downloads the swarm configuration file from the specified URL.
    In offline mode, it reads the swarm configuration file from the local disk.
    """
    if params_.offline_swarm:
        return read_file('swarm.csv', 'local CSV file', self.hw_id)
    else:
        print("Loading swarm configuration from online source...")
        try:
            print(f'Attempting to download file from: {params_.swarm_url}')
            response = requests.get(params_.swarm_url)

            if response.status_code != 200:
                print(f'Error downloading file: {response.status_code} {response.reason}')
                return None

            with open('online_swarm.csv', 'w') as f:
                f.write(response.text)

            return self.read_file('online_swarm.csv', 'online CSV file', self.hw_id)

        except Exception as e:
            print(f"Failed to load online swarm configuration: {e}")
    
    print("Swarm configuration not found.")


def read_file(filename, source, hw_id):
    with open(filename, newline='') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if int(row['hw_id']) == int(hw_id):
                print(f"Configuration for HW_ID {hw_id} found in {source}.")
                return row
    return None

if __name__ == '__main__':
    read_swarm()