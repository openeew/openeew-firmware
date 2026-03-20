# OpenEEW Firmware and ESP-IDF Refactor
## Overview

This feature branch introduces a refactor of the existing superloop-based architecture to a real-time operating system (RTOS), specifically **ESP-IDF FreeRTOS**.

The refactor aims to:

 - Improve system scalability and modularity

 - Optimize memory allocation and utilization

 - Enhance data quality and reliability

 - Increase system features through parallel task execution

 - Improve reliability of the system by utilizing task isolation

The updated codebase follows concurrency best practices. Inter-task communication is handled via queues, while shared resources are managed using locks and atomic operations to ensure thread safety.

## Task Architecture

Core device operations are implemented as independent tasks. Each task is assigned a priority level based on its criticality.

Since the controller is multi-core, non-critical tasks (e.g., system monitoring) are offloaded to the secondary core to improve overall performance.

Runtime profiling will be conducted to monitor heap utilization and identify optimization opportunities. Memory usage will be kept minimal to ensure system stability.

Given that the device operates remotely, additional precautions are taken to ensure reliable and stable system behavior under varying conditions.

## Implemented Tasks
### 1. Sensor Read Task

This task reads data from the accelerometer, applies necessary filtering, and pushes the processed data into the sensor data queue.

### 2. Sensor Data Publishing Task

This task retrieves sensor data from the queue and publishes it over MQTT.

Currently, data ordering is not guaranteed, as noted in the existing documentation:
https://github.com/openeew/openeew/tree/master/data

To address this, MQTT Quality of Service (QoS) level 2 is used to ensure reliable delivery. This implementation is expected to produce a consistent 1-second time step in the cloud.

QoS level 2 is particularly important for historical data analysis, as it minimizes data loss. This reduces the need for interpolation and estimation, thereby improving accuracy for data users (e.g., scientists).

Further architectural improvements are under consideration.

### 3. System State Publishing Task

This task publishes the state of various system modules, providing visibility into device activity.

Users can remotely enable or disable this feature via system commands to optimize cloud usage and associated costs.

### 4. Command Processing Task

This task processes user commands received over Ethernet or Wi-Fi and executes the corresponding actions.

In version 1 of the firmware, the following features are supported:

- Restart system modules

- Reconfigure MQTT parameters (e.g., broker address)

- Reconfigure Wi-Fi settings

- Enable system state publishing

### 5. System Monitoring Task

This task monitors system health, including:

- Sensor status

- Heap utilization and memory allocation

It can automatically trigger system state publishing when memory usage exceeds user-defined thresholds.

## Release Plan

These features will be included in version 0.1 of the RTOS-based firmware.

Further updates and enhancements will be implemented based on testing results and user feedback.