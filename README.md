# GFR RTK Package<br>
This is a repo for configuring a raspberry pi to be used as an NTRIP client, that connects to a NTRIP base station, recieves incoming RTCM data, and forwards it to an XSENS MTI 680g.

# Initializing the Package <br>
This configuration was created using a raspberry pi 4, and the repo must be cloned into a directory titled "boot" made on the path /home/gfrpi/boot/(repo contents).
If you make changes to the names of these directories you'll need to change the paths in each file accordingly.<br><br>
Both boot_script.sh and reset_counter.sh should be made executable with chmod +x filename.sh<br><br>
To get the script to run on boot, you must add this line to the crontab file with the command crontab -e<br>
```
@reboot /home/gfrpi/boot/boot_script.sh
```
These credentials and settings should be set to your specific information. Currently its set to connect to the Albany base station with GFR login. Make sure that your usb port is correct as well
```
        caster_ip="167.131.109.57",      # ODOT IP
        caster_port=9879,                 # ODOT port
        mountpoint="LCS1_GNSS",           # Albany MP
        username="GFR1",                  #  username
        password="GFR1",                   # pass 
        xsens_port="/dev/ttyUSB0",         # Xsens usb port
        baudrate=115200,                   # baudrate for serial bus
        log_file="/home/gfrpi/boot/ntrip_session.log"  # log file path
```

# Hardware Setup<br>
Step 1: Connect the raspberry pi to internet via ethernet or wifi<br>
Step 2: Connect the Xsens to the raspberry pi via the serial bus port into a usb port of the raspberry pi<br>
Step 3: Connect the main power and data port in the Xsens into your rover (i.e the car OR a computer for testing) <br>
Step 4: Connect the GPS antenna to the Xsens<br>
Step 5: Connect the raspberry pi to power via usb-c<br><br>
Now the script should begin automatically, and youll see the output in ntrip_session.log or in the console