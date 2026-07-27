# Septentrio & Xsens Integration Stack

This setup integrates the Septentrio mosaic-H driver with an Xsens IMU heading conversion node.  
Its primary goal is:

1. to stream RTCMv3 corrections to the Septentrio receiver over an internal TCP socket and  
2. expose IMU heading measurements in degrees

for real-time visualization and performance comparison in Foxglove.

## 1. System Overview

The [`heading_stack.launch`](https://github.com/ikh-innovation/septentrio_gnss_driver/blob/feature/heading_test/launch/heading_stack.launch) file launches:

* [`rover.launch`](https://github.com/ikh-innovation/septentrio_gnss_driver/blob/feature/heading_test/launch/rover.launch): main driver launch file that initializes static TF transforms, loads configuration parameters from a YAML file, and starts the `septentrio_gnss_driver_node`.
* [`rtcm_bridge.py`](https://github.com/ikh-innovation/septentrio_gnss_driver/blob/feature/heading_test/scripts/rtcm_bridge.py): TCP bridge for RTCM corrections
* [`xsens_heading.py`](https://github.com/ikh-innovation/septentrio_gnss_driver/blob/feature/heading_test/scripts/xsens_heading.py): node converting Xsens orientation (yaw) from quaternions to Euler angles

## 2. About RTK settings in [`rover.yaml`](https://github.com/ikh-innovation/septentrio_gnss_driver/blob/feature/heading_test/config/rover.yaml)

ROSaic Parameters/RTK corrections are available in driver's [README.md](https://github.com/ikh-innovation/septentrio_gnss_driver).

**Option 1:** NTRIP# RTK
```c++
rtk_settings:
  ntrip_1:
    id: "NTR1"
    caster: "185.25.22.91"
    caster_port: 2101
    username: "test"
    password: "test"
    mountpoint: "IKH_ASPROPYRGOS_RTCM32"
    version: "v1"
    tls: false
    fingerprint: ""
    rtk_standard: "RTCMv3"
    send_gga: "sec1"
    keep_open: true
```

**Option 2:** IP Server# RTK
```c++
  ip_server_1:
    id: "IPS1"
    port: 28784
    rtk_standard: "RTCMv3"
    send_gga: "sec1"
    keep_open: true
```

Option 1 *is not available* for now, cause the custom NTRIP caster needs to be change.  
Option 2 *needs a bridge between septentrio and RTCM correction*.

## 3. Nodes & Topics

### 3.1 Why `rtcm_bridge.py` is needed ([Internet over USB](https://customersupport.septentrio.com/s/article/Internet-Over-USB))

The Septentrio receiver receives network access and internet connection via USB. However, standard NTRIP/RTCM streams received over ROS topics cannot be fed directly into the native USB serial interface without additional driver configurations.

* Subscribes: `/rtcm`
* Output: forwards raw binary RTCMv3 stream directly into the Septentrio native TCP port `10.42.0.127:28784`.

### 3.2 Why `xsens_heading.py` is needed

The raw Xsens IMU output provides orientation as a 3D Quaternion. This node converts the complex quaternion data into a direct, human-readable Yaw angle (0° to ±180°).

* Subscribes: `/aristos/imu/data`
* Publishes: `/aristos/imu/heading` (heading calculated in degrees).

## 4. Usage

To launch the full integration stack, run:

```bash
roslaunch septentrio_gnss_driver heading_stack.launch
```