import json
import time
from io import BytesIO
from elasticsearch import Elasticsearch
import rospy
from rf_msgs.msg import Wifi

# Elasticsearch client
es = Elasticsearch("http://localhost:9200")  # Update this with your Elasticsearch instance

# Function to deserialize the CSI data from the .bin file
def deserialize_csi_data(data):
    """Assumes the data is a serialized ROS Wifi message."""
    try:
        wifi_msg = Wifi()  # Assuming Wifi is the correct ROS message type
        wifi_msg.deserialize(data)
        return wifi_msg
    except Exception as e:
        print(f"Failed to deserialize CSI data: {e}")
        return None

# Function to index deserialized data into Elasticsearch
def index_to_elasticsearch(index_name, wifi_data):
    """Indexes the Wifi data to Elasticsearch."""
    try:
        # Prepare the document to index
        doc = {
            'signal_strength': wifi_data.signal_strength,  # Example data
            'timestamp': int(time.time()*1000)  # Use current time or a timestamp from the message
        }

        # Index the document into Elasticsearch
        es.index(index=index_name, document=doc)
        print(f"Data indexed to Elasticsearch: {doc}")
    except Exception as e:
        print(f"Failed to index data to Elasticsearch: {e}")

# Function to extract data from MinIO and process it
def process_minio_data(object_name, bucket_name="csi-data"):
    """Extracts data from MinIO, deserializes it, and indexes it in Elasticsearch."""
    try:
        # Fetch the object from MinIO (use existing minio_client)
        response = minio_client.get_object(bucket_name, object_name)
        data = response.read()

        # Deserialize the data
        wifi_data = deserialize_csi_data(data)
        
        if wifi_data:
            # Index the deserialized data into Elasticsearch
            index_to_elasticsearch("csi-index", wifi_data)
    except Exception as e:
        print(f"Error processing MinIO data: {e}")

# Query Elasticsearch
def query_elasticsearch(index_name):
    """Queries Elasticsearch for indexed data."""
    try:
        query = {
            "match_all": {}
        }
        response = es.search(index=index_name, query=query)
        print(f"Query Results: {json.dumps(response, indent=2)}")
    except Exception as e:
        print(f"Failed to query Elasticsearch: {e}")

# Example to process a specific .bin file from MinIO and query it
if __name__ == "__main__":
    # Assuming a specific file to process
    object_name = "csi_data_1727301692564.bin"  # Replace with actual object names

    # Process the CSI data from MinIO and index it in Elasticsearch
    process_minio_data(object_name)

    # Query the indexed data in Elasticsearch
    query_elasticsearch("csi-index")
