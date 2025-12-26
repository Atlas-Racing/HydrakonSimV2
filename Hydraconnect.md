# Hydraconnect: Remote Connection Guide

This guide details the "Hydraconnect" workflow, which enables hybrid development scenarios where team members can run the ROS 2 stack locally while connecting to a remote CARLA simulation server.

## Overview

By default, HydrakonSimV2 connects to `localhost`. To connect to a remote server (e.g., a powerful simulation rig or the tracking PC), you need to:
1.  Configure Fast DDS for network discovery.
2.  Launch the system with the `host` argument.

## 1. Network Configuration (Fast DDS)

ROS 2 uses Fast DDS (Data Distribution Service) for communication. To ensure nodes on your machine can discover nodes on the remote server (and vice-versa) over specific IP addresses, you must configure a discovery profile.

### Step 1: Create the Discovery File

Create a file named `fastdds_discovery.xml` in your **home directory** (`~/fastdds_discovery.xml`).

```bash
nano ~/fastdds_discovery.xml
```

Paste the following content into the file. **Crucially, ensure the `<address>` entries include the IPs of ALL machines participating in the network (your local machine AND the remote server).**

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds>
    <profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
        <participant profile_name="UnicastParticipant" is_default_profile="true">
            <rtps>
                <builtin>
                    <discovery_config>
                        <discoveryProtocol>SIMPLE</discoveryProtocol>
                    </discovery_config>
                    <initialPeersList>
                        <!-- Local Machine IP (Example) -->
                        <locator>
                            <udpv4>
                                <address>10.6.136.241</address>
                            </udpv4>
                        </locator>
                        <!-- Remote Server / CARLA PC IP -->
                        <locator>
                            <udpv4>
                                <address>10.6.137.88</address>
                            </udpv4>
                        </locator>
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
```

> **Note:** Replace `10.6.136.241` with your local IP and `10.6.137.88` with the remote server's IP.

### Step 2: Set Environment Variables

You need to tell ROS 2 to use this configuration file and ensure all machines are on the same Domain ID.

Run the following commands in your terminal (or add them to your `~/.bashrc` for persistence):

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_discovery.xml
export ROS_DOMAIN_ID=0
```

To add them to `.bashrc` so they load automatically:
```bash
echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=~/fastdds_discovery.xml' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

## 2. Launching the Simulation

Once the network is configured, you can launch the Hydrakon stack and point it to the remote CARLA server.

### Connect to Remote Server

Use the `host` argument to specify the IP address of the machine running CARLA:

```bash
ros2 launch hydrakon_launch launch.py host:=10.6.137.88
```

This command will:
1.  Connect the **Vehicle Spawner**, **Camera Spawners**, and **LiDAR** nodes to the CARLA instance at `10.6.137.88`.
2.  Receive sensor data and publish ROS 2 topics locally on your machine.
3.  Launch RViz locally to visualize the remote simulation.

### Connect to Localhost (Default)

If running everything on one machine, simply omit the `host` argument:

```bash
ros2 launch hydrakon_launch launch.py
```

## 3. Other Workflows

The `host` argument is supported across all major launch files, allowing you to run mapping or navigation stacks locally while interacting with a remote CARLA instance.

### Mapping Lap (SLAM)

To run SLAM and generate a map from a remote simulation:

```bash
ros2 launch hydrakon_launch mapping_lap.launch.py host:=10.6.137.88
```

### Navigation (Nav2)

To run the full Navigation 2 stack with a remote simulation:

```bash
ros2 launch hydrakon_launch navigation.launch.py host:=10.6.137.88
```
