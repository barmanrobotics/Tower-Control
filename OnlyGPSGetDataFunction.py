#import the necessary libraries
import gps
import time

# make sure to have gps: sudo apt-get install gpsd gpsd-clients python3-gps

# GPS setup 
gpsd = gps.gps(mode=gps.WATCH_ENABLE) # Start the GPS (assuming GPS is connected and working)

# Function to get GPS data
def get_gps_data():
    try:
        # Wait for a fix
        while True:
            gpsd.next()  # this will continue to loop until we get a fix
            if gpsd.fix.mode >= 2:  # Check if we have a 2D or 3D fix
                break
        # Return the latitude and longitude
        return gpsd.fix.latitude, gpsd.fix.longitude
    except KeyboardInterrupt:
        print("Exiting GPS data retrieval.")
        return None