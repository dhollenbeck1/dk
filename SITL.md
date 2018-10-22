# Getting Dronekit working

Follow the procedure below to get started with dronekit.

## Useful links

* http://python.dronekit.io
* http://python.dronekit.io/examples/index.html
* https://www.learnpython.org/en/Variables_and_Types
* https://www.learnpython.org/en/Functions
* http://python.dronekit.io/guide/quick_start.html
* http://python.dronekit.io/guide/index.html

# PRELAB / CLASS

## Download Anaconda 2
For python2.7: https://www.anaconda.com/download/
Open Anaconda prompt and navigate to your working directory. Use

    cd <foldername> # to down a folder level 
    cd .. #to move up a folder level
    
## Download Mission Planner

For Windows: http://ardupilot.org/planner/docs/mission-planner-installation.html
Open Mission planner.
Change the connection method (upper right hand corner) to TCP and follow the rest of the steps.

## Download dependencies
In the Anaconda Prompt type:
    
    pip install dronekit
    pip install dronekit-sitl
    
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
