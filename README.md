# WiSenseHub: Scalable WiFi Sensing System

## Introduction:
WiSenseHub is a scalable system designed to collect, process, and store WiFi and user device data using a containerized architecture. The system combines several distinct elements to effectively manage data streams from numerous Raspberry Pi (RPI) devices placed across a building-scale environment.


## Table of Contents:
1. [Key Features](#key-features)
2. [Usage](#usage)
3. [Set Up](#set-up)
4. [Citations](#citations)

### Key Features:
* **MQTT-based Communication**: 
Raspberry Pi devices send WiFi data through an MQTT protocol, where the server acts as the central subscriber collecting data from multiple publishers (RPIs).
* **Dockerized Deployment**: 
The system is deployed using Docker to ensure easy scalability and portability. By containerizing the architecture, WiSenseHub can be easily managed and scaled to handle building-wide data collection.
* **Pre-Processing Pipeline**: 
A Python-based pre-processing pipeline is integrated to process and clean the WiFi data received from the RPIs, preparing it for long-term storage and analysis.
* **MinIO for Data Storage**: 
The system uses MinIO, an open-source alternative to AWS S3, to provide high-performance storage for large volumes of streaming data. MinIO organizes and stores data locally, enabling seamless scalability.
* **ElasticSearch for Querying**: 
For advanced querying and analysis, WiSenseHub leverages ElasticSearch instead of DynamoDB and Lambda functions. ElasticSearch indexes both WiFi and user device data, allowing for efficient querying and data analysis.

### Usage:

### Set Up:

### Citations:

