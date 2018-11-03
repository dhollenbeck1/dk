# Getting Dronekit working

Follow the procedure below to get started with dronekit.

## Useful links

* https://winscp.net/eng/download.php
* http://python.dronekit.io
* https://www.learnpython.org
* https://commandwindows.com/command3.htm
* https://www.anaconda.com/download/#windows
* https://notepad-plus-plus.org/download/v7.5.1.html
* https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html

# PRELAB / CLASS

## Download Anaconda 2
For python2.7: https://www.anaconda.com/download/
Open Anaconda prompt and navigate to your working directory. Use

    cd <foldername> # to down a folder level 
    cd .. #to move up a folder level
    
## Download Mission Planner

For Windows: http://ardupilot.org/planner/docs/mission-planner-installation.html
* Open Mission planner. 
* Click on simulation and type the following command into the extra command line field

    --home=37.3769269491065,-120.413997173309,87.422486744914,0
    
This will start the SITL with the --home=lat,lon,alt(MSL),absolute heading
    
## Download dependencies
In the Anaconda Prompt type:
    
    pip install dronekit
    pip install dronekit-sitl
    pip install mavproxy
  
If mavproxy works for you. You can use the following commands and connect on MP through the UDP port below.
* mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out udp:10.55.222.120:14550
    
## Run example 
Copy the below code and save as python file in Spyder. 

    print( "Start simulator (SITL)" )
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

    # Import DroneKit-Python
    from dronekit import connect, VehicleMode

    # Connect to the Vehicle.
    print("Connecting to vehicle on: %s" % (connection_string,))
    vehicle = connect(connection_string, wait_ready=True)

    # Get some vehicle attributes (state)
    print ("Get some vehicle attribute values:")
    print (" GPS: %s" % vehicle.gps_0)
    print (" Battery: %s" % vehicle.battery)
    print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
    print (" Is Armable?: %s" % vehicle.is_armable)
    print (" System status: %s" % vehicle.system_status.state)
    print (" Mode: %s" % vehicle.mode.name )   # settable

    # Close vehicle object before exiting script
    vehicle.close()

    # Shut down simulator
    sitl.stop()
    print("Completed")

## Other Examples

Download or pull from the Dronekit-python library in Github. From the git shell and Linux you can type:

    git clone http://github.com/dronekit/dronekit-python.git
