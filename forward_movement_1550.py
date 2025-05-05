from dronekit import connect, VehicleMode

from time import sleep


# Connect to the Vehicle.

connection_string="192.168.3.10:14551"

print("Connecting to vehicle on: %s" % (connection_string,))

vehicle = connect(connection_string, wait_ready=False)



# Get some vehicle attributes (state)

print ("Get some vehicle attribute values:")

print (" GPS: %s" % vehicle.gps_0)

print (" Battery: %s" % vehicle.battery)

print (" System status: %s" % vehicle.system_status)

print (" Mode: %s" % vehicle.mode.name)

print ("%s" % len(vehicle.channels))    



# Arm to move the robot

# vehicle.armed=True

# print ("%s" % vehicle.armed)



while vehicle.armed==True:

	vehicle.armed=False

	sleep(0.1)

	print ("%s" % vehicle.armed)



vehicle.armed=True

sleep(0.1)

print ("%s" % vehicle.armed)

while True:

#	sleep(5)



#	vehicle.channels.overrides ['5']=1600

#	sleep(20)

#	vehicle.channels.overrides ['5']=1500

#	sleep(60)



#	vehicle.channels.overrides ['5']=1600

#	sleep(5)

#	vehicle.channels.overrides ['5']=1500

#	sleep(5)



#	vehicle.channels.overrides ['5']=1650

#	sleep(20)

#	vehicle.channels.overrides ['5']=1500

#	sleep(60)



#	vehicle.channels.overrides ['5']=1650

#	sleep(5)

#	vehicle.channels.overrides ['5']=1500

#	sleep(5)



#	vehicle.channels.overrides ['5']=1700

#	sleep(20)

#	vehicle.channels.overrides ['5']=1500

#	sleep(60)



#	vehicle.channels.overrides ['5']=1700

#	sleep(5)

#	vehicle.channels.overrides ['5']=1500

#	sleep(5)



#	vehicle.channels.overrides ['5']=1750

#	sleep(20)

#	vehicle.channels.overrides ['5']=1500

#	sleep(60)



	vehicle.channels.overrides ['5']=1550

	sleep(3)

	vehicle.channels.overrides ['5']=1500

	sleep(5)



#	vehicle.channels.overrides ['5']=1800

#	sleep(5)

#	vehicle.channels.overrides ['5']=1500

#	sleep(5)



#	vehicle.channels.overrides ['5']=1800

#	sleep(20)

#	vehicle.channels.overrides ['5']=1500

#	sleep(60)







	# Motor values

	# 1000 reverse full speed

	# 1500 middle (off)

	# 2000 forward full speed



	# Motor channels for vehicle control

	#1 Pitch

	#2 Roll

	#3 Throttle

	#4 Yaw

	#5 Forward

	#6 Lateral

	break

vehicle.channels.overrides ['5']=1500

