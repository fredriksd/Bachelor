#-*- coding:utf-8 -*-
#!/usr/bin/python


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import argparse
from pixy_spi import PWM, get_Pixy #<-- Noe i denne duren blir det.

#Parametre 
lost = False
lost_counter = 0

####RC####
ROLL = '1'
PITCH = '2'
THROTTLE = '3'
NAV_MODE = "ALT_HOLD"
pwm_roll = PWM(P_gain = 200, D_gain = 200, inverted = True) 
pwm_pitch = PWM(P_gain = 200, D_gain = 200, inverted = True)
#####

####PIXEL####
x_max = 319
x_min = 0
y_max = 199
y_min = 0

x_center = (x_max - x_min)/2
y_center = (y_max - y_min)/2
####

is_takeover = False

#SETUP 
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/serial0',
	help = 'Connect to vehicle on ip address given. Default set to 127.0.0.1:14550')
args = parser.parse_args()

# Koble til fartøy
print'Connecting to vehicle on: %s' % args.connect
vehicle = connect(args.connect, wait_ready=True, heartbeat_timeout=15)

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



def takeover():
  '''Funksjon for å sjekke for takeover fra radio eller GCS.'''
  if ROLL and PITCH in vehicle.channels.overrides and not vehicle.mode.name == NAV_MODE:
    return True
  else:
    return False

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


def manual_flight():
  '''
  Funksjon for manuell flyging. Programmet står i ro mens 
  dronen er kontrollert manuelt. Når vericle.mode.name == NAV_MODE gå tilbake til pixy_search
  '''
  while vehicle.mode.name != NAV_MODE:
    print "Manual flight..."
    time.sleep(0.3)

'''
Lage en pixy_search funksjon som søker etter pixy OG sjekker swith på kontroller. 
Hvis vericle.mode.name != NAV_MODE, gå inn i manual_fligh funksjon. Når pixy ser objekt, gå videre i "while true" løkka
'''
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
    
    arm_and_takeoff(20)
    #is_takeover = takeover()
    vehicle.mode = VehicleMode(NAV_MODE)
    vehicle.channels.overrides[THROTTLE] = 1500 #BARE FOR SITL
    time.sleep(0.5)

    if vehicle.mode.name == NAV_MODE and vehicle.parameters["ANGLE_MAX"] == 4500:
      vehicle.parameters["ANGLE_MAX"] = 1000

    
    while True:
      if not takeover():
        #Dersom gjenoppretting av lyspunkt:
        #Skift tilbake til NAV_MODE og redusere vinkelutslag
        if lost:
          vehicle.mode = VehicleMode(NAV_MODE)
          lost_counter = 0
          vehicle.parameters["ANGLE_MAX"] = 1000 

        send = get_Pixy()
        while not send:
          #lag en pixy_lost funksjon
          print 'Searching...'
          send = get_Pixy()
          lost_counter += 1
          time.sleep(0.01)

          if ROLL and PITCH in vehicle.channels.overrides and lost_counter == 15:
            vehicle.mode = VehicleMode("LOITER")
            vehicle.parameters["ANGLE_MAX"] = 4500
            lost = True

        print send

        #Beregner feil for PD-regulering
        error_x = x_center - send[0]
        error_y = y_center - send[1]
        #Setter denne feilen inn i PD-regulatoren og beregner PWM-posisjon
        pwm_roll.update(error_x)
        pwm_pitch.update(error_y)

        print vehicle.channels
      
        print "Roll: ", pwm_roll.position
        print "Pitch: ", pwm_pitch.position
        
        vehicle.channels.overrides = {THROTTLE: 1500, ROLL: pwm_roll.position ,PITCH: pwm_pitch.position}
        time.sleep(0.1)
      else:
        '''
        Ved takeover fra senderen (som merkes i form av mode-bytte) renses kanaloverskrivelsene
        slik at en kan gjenopprette kontrollen.
        '''
        print "Tx takeover"
        vehicle.channels.overrides = {THROTTLE: 1500} #BARE FOR SITL
        vehicle.parameters["ANGLE_MAX"] = 4500
        manual_flight()
        


  # Close vehicle object
  except KeyboardInterrupt:
    vehicle.mode = VehicleMode('LOITER')
    vehicle.channels.overrides = {THROTTLE: 1500}
    #vehicle.armed = False
    vehicle.close()
  
  vehicle.close()