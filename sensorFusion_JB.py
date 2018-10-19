import oculus
import viz
import vizact
import viztracker
import datetime
import vizmat
#import numpy as np



init = False
# add intersense tracker
isense = viz.add('intersense.dle')
while not init:
	ISTracker = isense.addTracker(port=5001,station=0)
	if ISTracker.getPosition()[0] != 0:
		init = True
ISTracker.setEnhancement(2) 
ISTracker.setSensitivity(4)
ISTracker.setShockSuppression(2)
ISTracker.setAccelSensitivity(4)


# add Oculus tracker
OVRTracker = oculus.Rift().getSensor()

# add the virtual tracker, link it to the MainView and set an offset to get the eye position
virtual = viz.addGroup()
link = viz.link(virtual, viz.MainView)
# setting position offset
link.preTrans([0, -0.055, -0.073])

viz.go()


# variables for recording
DATA_COLLECT = True
dataBuffer = ''
time = 0
MODEL_DIR = 'Models/'

OUTPUT_DIR = 'sensorFusionTestData/'
frame = 0
timeID = datetime.datetime.now().strftime('%m%d%y%H%M%S')
fusionInit = False

piazza = viz.addChild('piazza.osgb')



def masterLoop(num):
	global time, timeID, DATA_COLLECT, MODEL_DIR, OUTPUT_DIR, dataBuffer, frame, OVRTracker, virtual, \
	ISTracker, init, fusionInit, ovrEulerOld
	
	# record time
	time += viz.getFrameElapsed()
	frame += 1
	# first frame of tracker data	
	if init and not fusionInit:
		virtual.setEuler(ISTracker.getEuler())
		ovrEulerOld = OVRTracker.getEuler()
		fusionInit = True
		
	# after first frame of tracker data
	if fusionInit:
		# read trackers updataes
		ovrEuler = OVRTracker.getEuler()
		ISQuat = ISTracker.getQuat() # quaternion
		
		# compute the orientation (as quaternion) if only use the update of oculus 
		virtualQuatByOvr = vizmat.EulerToQuat([a + b - c for a, b, c in zip(virtual.getEuler(), ovrEuler, ovrEulerOld)])
		
		# use intersense position
		virtual.setPosition(ISTracker.getPosition())
		
		# set camera orientation as the slerp from the oculus-updated orientation to intersense orientation
		virtual.setQuat(vizmat.slerp(virtualQuatByOvr, ISQuat, viz.getFrameElapsed()))
	
		# save the oculus orientation for next frame
		ovrEulerOld = ovrEuler

	
	if DATA_COLLECT:
		# write data to buffer		
		data = OVRTracker.getEuler() + ISTracker.getEuler() + virtual.getEuler()
		strData = [str(round(t,4)) for t in data+[time]]
		strData = ','.join(strData)+'\n'
		dataBuffer = dataBuffer + strData
		

		# write buffer to file
		if frame % 450 == 0:
			fileName = OUTPUT_DIR + 'sensorfusion' + timeID + '.csv'
			file = open(fileName, 'a')
			file.write(dataBuffer)
			dataBuffer = ''
			file.close()
			print(data)


# Restarts the loop, at a rate of 90 Hz
viz.callback(viz.TIMER_EVENT,masterLoop)
viz.starttimer(0,1.0/90,viz.FOREVER)





