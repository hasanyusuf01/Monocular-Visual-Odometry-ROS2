
### build a package 
colcon build --packages-select pkgname --symlink-install

### run a package node 

ros2 run pkg_name node_name

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