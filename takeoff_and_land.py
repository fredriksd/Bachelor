#-*- coding:utf-8 -*-

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse  


#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='udp:127.0.0.1:14550',
	help = 'Connect to vehicle on ip address given. Default set to 127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, baud=57600, wait_ready=True)

''' #Callback to print the location in global frames. 'value' is the updated value
def location_callback(self, attr_name, value):
   print "Location (Global): ", value

'''

last_rangefinder_distance=0

@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
    #attr_name not used here.
    global last_rangefinder_distance
    if last_rangefinder_distance == round(self.rangefinder.distance, 1):
        return
    last_rangefinder_distance = round(self.rangefinder.distance, 1)
    print " Rangefinder (metres): %s" % last_rangefinder_distance

@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print " CALLBACK: Mode changed to", value

#print " Set mode=STABILIZE (currently: %s) and wait for callback" % vehicle.mode.name 

def set_home():
  while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print "Waiting for home location..."
  
  print "Home location set"
  time.sleep(1)
  print "Home location set at: {}".format(vehicle.home_location)
    #print "Home location set at: {}".format(vehicle.home_location)
    

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print " Waiting for vehicle to initialise..."
    time.sleep(1)

    while vehicle.gps_0.fix_type < 2:
        print "Waiting for GPS", vehicle.gps_0.fix_type
        time.sleep(1)
    
  print "Arming motors"
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print " Waiting for arming..."
    time.sleep(1)

  print "Taking off!"
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print "Altitude: ", vehicle.location.global_relative_frame.alt 
    #Break and return from function just below target altitude.        
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print "Reached target altitude"
      break
    time.sleep(1)

def simpleGoto(lat, longi):
	loc = LocationGlobalRelative(lat, longi)
	print "moving to ({}, {})".format(lat, longi)
	vehicle.simple_goto(loc)

#get_start_status()
#Funksjon for å hente ut data om UAV's tilstand før vi flyr
#for å bestemme om det er greit å fly.


#MAIN PROGRAM
if __name__ == "__main__":
	# Initialize the takeoff sequence to 20m
	set_home()
	arm_and_takeoff(20)


	# Hover for 10 seconds
	time.sleep(10)
	print "Entering LAND mode"
	vehicle.mode = VehicleMode("LAND")


# Close vehicle object
vehicle.close()