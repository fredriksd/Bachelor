#-*- coding:utf-8 -*-
#!/usr/bin/python


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
from pixy_spi import PWM, get_Pixy #<-- Noe i denne duren blir det.


x_max = 319
x_min = 0
y_max = 199
y_min = 0

x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2


#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/serial0',
	help = 'Connect to vehicle on ip address given. Default set to 127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True)

''' #Callback to print the location in global frames. 'value' is the updated value
def location_callback(self, attr_name, value):
   print "Location (Global): ", value

'''



last_rangefinder_distance=0
'''
@vehicle.on_attribute('rangefinder')
def rangefinder_callback(self,attr_name):
    #attr_name not used here.
    global last_rangefinder_distance
    if last_rangefinder_distance == round(self.rangefinder.distance, 1):
        return
    last_rangefinder_distance = round(self.rangefinder.distance, 1)
    print " Rangefinder (metres): %s" % last_rangefinder_distance
'''
@vehicle.on_attribute('mode')   
def decorated_mode_callback(self, attr_name, value):
    # `attr_name` is the observed attribute (used if callback is used for multiple attributes)
    # `attr_name` - the observed attribute (used if callback is used for multiple attributes)
    # `value` is the updated attribute value.
    print "CALLBACK: Mode changed to", value


#print " Set mode=STABILIZE (currently: %s) and wait for callback" % vehicle.mode.name 

def set_home():
  while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    print"Waiting for home location..."
  
  print "Home location set"
  time.sleep(1)
  print "Home location set at: {}".format(vehicle.home_location)

    
def arm():
  print "Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
#  while not vehicle.is_armable:
#    print " Waiting for vehicle to initialise..."
#    time.sleep(1)

#    while vehicle.gps_0.fix_type < 2:
#        print "Waiting for GPS", vehicle.gps_0.fix_type
#        time.sleep(1)
    
  print "Arming motors"
  # Copter should arm in GUIDED mode
  #vehicle.mode    = VehicleMode("GUIDED")
  vehicle.mode = VehicleMode("STABILIZE")
  vehicle.armed   = True

  while not vehicle.armed:
    print "Waiting for arming..."
    time.sleep(1)
  

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print"Basic pre-arm checks"
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print "Waiting for vehicle to initialise..."
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
  try:
    arm()
    #vehicle.mode = VehicleMode("STABILIZE")
    if vehicle.mode.name == "STABILIZE" and vehicle.parameters["ANGLE_MAX"] == 4500:
      vehicle.parameters["ANGLE_MAX"] = 1000

    pwm_roll, pwm_pitch = PWM(P_gain = 100, D_gain = 200, inverted = True), PWM(P_gain = 100, D_gain = 200)

    '''Ved takeover fra senderen (som merkes i form av mode-bytte) renses kanaloverskrivelsene
    slik at en kan gjenopprette kontrollen.
    '''
    while True:
      if vehicle.channels.overrides and not vehicle.mode.name == "STABILIZE":
        print "Tx takeover"
        vehicle.channels.overrides = {}
        break

      send = get_Pixy()
      while not send:
        print 'Searching...'
        time.sleep(0.01)
        send = get_Pixy()
        if vehicle.channels.overrides:
          vehicle.channels.overrides = {}
          vehicle.mode = VehicleMode("LOITER")
      print send

      #Beregner feil for PD-regulering
      error_x = x_center - send[0]
      error_y = y_center - send[1]
      #Setter denne feilen inn i PD-regulatoren og beregner PWM-posisjon
      pwm_roll.update(error_x)
      pwm_pitch.update(error_y)

      print vehicle.channels
      
      #Dette er visst den anbefalte metoden for å sende mavlink-kommandoer over pymavlink
      #vehicle.send_mavlink(roll_msg)
      #vehicle.send_mavlink(pitch_msg)
      
      print "Roll: ", pwm_roll.position
      print "Pitch: ", pwm_pitch.position
      
      vehicle.channels.overrides = {'1': 1100, '2': pwm_roll.position , '3': pwm_pitch.position}

  # Close vehicle object
  except KeyboardInterrupt:
    vehicle.channels.overrides = {}
    vehicle.mode = VehicleMode('GUIDED')
    vehicle.armed = False
    vehicle.close()
  
  vehicle.close()
'''
  throttle_msg = vehicle.message_factory.command_long_encode(
    0,0,
    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
    0,
    1,
    1300,
    0,0,0,0,0
    ) 
  motor_test = vehicle.message_factory.command_long_encode(
    0,0,
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
    0,
    1,
    1,
    1200,
    10,
    0,
    2,
    0
  )
  #vehicle.send_mavlink(motor_test)
  vehicle.send_mavlink(throttle_msg)

        roll_msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        2,
        pwm_roll.position,
        0,0,0,0,0
      )
      pitch_msg = vehicle.message_factory.command_long_encode(
        0,0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        3,
        pwm_pitch.position,
        0,0,0,0,0
      )
'''
