# WiSenseHub: Scalable WiFi Sensing System

## Introduction:
WiSenseHub is a scalable system designed to collect, process, and store WiFi and user device data using a containerized architecture. The system combines several distinct elements to effectively manage data streams from numerous Raspberry Pi (RPI) devices placed across a building-scale environment.


## Table of Contents:
1. [Key Features](#key-features)
1. [Important Files](#important-files)
2. [Configuration and Setup](#configuration-and-setup)
3. [Usage Examples](#usage-examples)
4. [Citations](#citations)


### Key Features

- **Real-Time Data Collection**  
  WiSenseHub collects real-time WiFi data from Raspberry Pi devices. The data is streamed continuously using the MQTT protocol, ensuring low latency and up-to-the-second updates.

- **MQTT-Based Communication**  
  WiSenseHub facilitates seamless communication between Raspberry Pi devices and the server using the MQTT protocol. Raspberry Pis act as publishers, sending WiFi data, while the server operates as a central subscriber, efficiently aggregating data from multiple sources.

- **Data Upload to MinIO Buckets and attained high-performance storage**  
  The real-time data received from the Raspberry Pi devices is automatically uploaded to MinIO buckets for secure and efficient storage. MinIO serves as a high-performance, scalable solution for managing large volumes of streaming data.
  Leveraging MinIO, an open-source alternative to AWS S3, the system provides efficient storage for large-scale streaming data. MinIO organizes and stores data locally, offering seamless scalability and high performance.

- **Dockerized Deployment**  
  The system is fully containerized using Docker, ensuring easy deployment, scalability, and portability. This architecture simplifies building-wide data collection and management while enabling rapid scaling to handle increasing data volumes.

- **Python Pre-Processing Pipeline**  
  A robust Python-based pre-processing pipeline cleans and processes WiFi data received from Raspberry Pi devices. This ensures the data is optimized for long-term storage and analysis.

- **ElasticSearch for Advanced Querying**  
  WiSenseHub integrates ElasticSearch for powerful querying and analysis. ElasticSearch indexes both WiFi and user device data, enabling efficient, real-time data exploration and advanced insights, replacing traditional DynamoDB and Lambda solutions.

- **Scalable Architecture**  
  The system’s modular design ensures easy scalability, allowing for the addition of new Raspberry Pi devices or topics without extensive reconfiguration.

- **Secure Data Transmission**  
  Implements industry-standard encryption protocols such as TLS/SSL for secure data transmission between Raspberry Pi devices and the central server.

- **Support for Custom Topics**  
  Enables flexibility by supporting custom MQTT topics for additional types of sensor data, making it adaptable to various IoT use cases.

- **Automated Alert System**  
  Configurable alerts can be set up for specific events or anomalies detected in the data stream, ensuring prompt notifications for critical insights.

- **Fault-Tolerant System**  
  Designed with robust error-handling mechanisms to ensure continuous operation and automatic recovery in case of hardware or network failures.


### Important Files

These scripts form a complete pipeline:  
1. **Data Collection**: `ros_publisher.py` streams real-time data from ROS to ZMQ.  
2. **Data Storage**: `ros_minio_publisher.py` enables simultaneous real-time streaming and storage in MinIO.  
3. **Data Querying and Analysis**: `minio_elasticsearch_handler.py` facilitates retrieval, parsing, indexing, and querying of stored data for insights and visualization.  

## **Configuration and Setup**

To set up and run the code on your local server, follow these steps. The configuration and setup are tailored for a local environment where we are using MinIO for storage, Elasticsearch for querying, and ZMQ for communication.

### **Prerequisites**

Ensure the following software is installed and properly configured on your local server:

1. **ROS (Robot Operating System)**: Ensure you have ROS installed. The code uses `rospy` to interact with ROS topics.
   - For installation instructions, visit [ROS Installation Guide](http://wiki.ros.org/ROS/Installation).
   
2. **MinIO**: This is an open-source object storage service used to store data. It is configured locally for this setup.
   - You can follow the installation instructions from [MinIO's official guide](https://docs.min.io/docs/minio-quickstart-guide).

3. **Elasticsearch**: This is used for indexing and querying data.
   - Install Elasticsearch by following the official guide: [Installing Elasticsearch](https://www.elastic.co/guide/en/elasticsearch/reference/current/install-elasticsearch.html).

4. **ZMQ (ZeroMQ)**: For communication between your ROS nodes and the subscriber, install `zmq` on your local server.
   - Install `pyzmq` for Python by running:  
     ```
     pip install pyzmq
     ```

5. **Python Libraries**: The following Python libraries are required for running the code:
   - `rospy`
   - `zmq`
   - `minio`
   - `elasticsearch`
   - `struct`
   - `rf_msgs`
     
### Configuration Steps

### MinIO Client Configuration

```python

from minio import Minio
# MinIO Client Configuration
minio_client = Minio(
    "localhost:1200",
    access_key="value",
    secret_key="value",
    secure=False # Set to True if using HTTPS
)```


### Usage:

### Citations:

