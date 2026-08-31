
### build a package 
colcon build --packages-select pkgname --symlink-install

### run a package node 

ros2 run pkg_name node_name




### list all active topics 

 ros2 topic list
/camera/camera_info
/camera/image_raw/compressed
/mobile_sensor/gps
/mobile_sensor/imu
/mobile_sensor/pose
/mobile_sensor/speech
/mobile_sensor/tts
/mobile_sensor/tts_wav
/mobile_sensor/wav_bytes
/parameter_events
/rosout
### list the active topic message type 

yusuf@LAPTOP-LUNJNS0R:~/workSp$ ros2 topic type /mobile_se
nsor/imu
sensor_msgs/msg/Imu

### list the interface format 

yusuf@LAPTOP-LUNJNS0R:~/workSp$ ```  ros2 interface show sensor_msgs/msg/Imu ```

```

# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

geometry_msgs/Quaternion orientation
        float64 x 0
        float64 y 0
        float64 z 0
        float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
        float64 x
        float64 y
        float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
        float64 x
        float64 y
        float64 z
float64[9] linear_acceleration_covariance # Row major x, y z

```


## Step 1: Open Port 4000 in Windows Firewall

You need to allow outside devices to talk to Windows on port 4000.

1. Open **PowerShell** on Windows as an **Administrator** (right-click the Start button -> Terminal (Admin) or Windows PowerShell (Admin)).
2. Run this command:
```powershell
New-NetFirewallRule -DisplayName "WSL2 Mobile Sensor Port 4000" -Direction Inbound -LocalPort 4000 -Action Allow -Protocol TCP

```



## Step 2: Set up Port Forwarding (PortProxy)

Now, tell Windows to forward any traffic hitting its port 4000 straight into WSL.

Keep your Administrator PowerShell open, and run this single block of code (copy and paste the whole thing):

```powershell
$wsl_ip = (wsl -- hostname -I).Trim()
netsh interface portproxy add v4tov4 listenport=4000 listenaddress=0.0.0.0 connectport=4000 connectaddress=$wsl_ip

```

*(Note: Because the WSL IP address changes every time you restart your PC, you will have to re-run this specific `netsh` command after a reboot.)*

## Step 3: Connect from your Phone

Your phone still cannot connect to `172.25...`. Instead, it needs to connect to your **Windows laptop's Wi-Fi IP address**.

1. In that same PowerShell window, find your laptop's IP address by running:
```powershell
ipconfig

```


2. Look for the `IPv4 Address` under your **Wireless LAN adapter Wi-Fi** (it usually looks like `192.168.1.X` or `10.0.0.X`).
3. On your phone, open your browser and type that IP address with port 4000, like this:
**`[https://192.168.1.](https://192.168.1.)X:4000`** *(replace with your actual IP)*

**Important:** Because this server uses a self-signed HTTPS certificate, your mobile browser will show a scary "Your connection is not private" warning. You must click "Advanced" and then "Proceed to 192.168... (unsafe)" to grant the camera and sensor permissions.




## to view the live feed 

```
 ros2 run rqt_image_view rqt_image_view

 ```