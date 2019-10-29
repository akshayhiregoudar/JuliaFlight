# using sensor - to get data from another code

# using IMU
using LinearAlgebra
using Statistics
# using Compat
# using Random
using PyPlot

# sensor = IMU.MPU9250()

gyroRoll = []
gyroPitch = []
gyroYaw = []

accX = []
accY = []
accZ = []

magX = []
magY = []
magZ = []

# Euler Angles

# Gyroscope

gyroEulerAngle = [[0],[0],[0]]
gyroEulerRoll = []
gyroEulerPitch = []
gyroEulerYaw = []

# Accelerometer

accEulerAngle = [[0],[0],[0]]
accEulerPitch = []
accEulerRoll = []
accEulerYaw = []

# Magnetometer

magEulerAngle = [[0],[0],[0]]
magEulerRoll = []
magEulerPitch = []
magEulerYaw =[]

dt = 0.01   # dt is the sensor data rate

# Define the variables of Kalman Filter

A = [1 0 0 -dt 0 0; 0 1 0 0 -dt 0; 0 0 1 0 0 -dt; 0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1];
B = [dt 0 0; 0 dt 0; 0 0 dt; 0 0 0; 0 0 0; 0 0 0];
C = [1 0 0 0 0 0; 0 1 0 0 0 0; 0 0 1 0 0 0];

P = eye(6);
Q = eye(6)*0.0008;
R = [0.1 0 0; 0 0.1 0; 0 0 10];

state_estimate = [0 0 0 0 0 0];

# The raw data is passed through the filter

filteredData = [[0],[0],[0]]
filteredRoll = []
filteredPitch = []
filteredYaw = []

# Save data (Optional)

save = string(input("Do you want to save the sensor data? Y/N "));
if (save == "y" or save == "Y")
	fileName = str(input("Enter the file name: "))
	mkdir("FileName")
	mkdir("FileName/ RawValues")
	mkdir("FileName/ AngleEstimates")
	mkdir("FileName/ Comparisions")
	mkdir("FileName/ Comparisions/ FilteredComparisions")
end
	

# Define functions to estimate the Euler Angles

function accCalEuler(sensorValues)
	phi = atand((sensorValues[5],hypot(sensorValues[4],sensorValues[6]))
	theta = atand((-1*sensorValues[4]),hypot(sensorValues[5],sensorValues[6]))
	psi = 0
	
	eulerEstimate = [[phi],[theta],[psi]]
	return eulerEstimate
end


function gyroCalEuler(sensorValues, prevEulerAngle)
	currentAngVel = [[sensorValues[1], sensorValues[2], sensorValues[3]]
	phi = prevEulerAngle.item((0,0))
	theta = prevEulerAngle.item((1,0))

	M = [[1, tan(theta)*sin(phi), tan(theta)*cos(phi)], [0, cos(phi), -1*sin(phi)], [0, sin(phi)/cos(theta), cos(phi)/cos(theta)]]
	eulerAngle = prevEulerAngle + M*currentAngVel*dt

	return eulerAngle
end

function magCalEuler(sensorValues, accEulerAngle, magCorrectionMean, gyroEstimate)
	phi = 0
	theta = 0
	accPhi = deg2rad(accEulerAngle.item(0,0))
	accTheta = deg2rad(accEulerAngle.item(1,0))
	psi = atan((sensorValues[9]*sin(accPhi) - sensorValues[8]*cos(accPhi)), (sensorValues[7]*cos(accTheta) + (sensorValues[8]*sin(accTheta)*sin(accPhi)) + (sensorValues[9]*sin(accTheta)*cos(accPhi)))
	
	if (magCorrectionMean == 0)
		psi = rad2deg(psi)
	else
		psi = rad2deg(psi) - magCorrectionMean
	end
	
	eulerEstimate2 = [[phi], [theta], [psi]]
	return eulerEstimate2
end


function CollectData(currentTime, gyroRollCorrection, gyrpPitchCorrection, gyroYawCorrection)
	accel = sensor.readAccel()
	gyro = sensor.readGyro()
	mag = sensor.readMagnet()
	push!(times, currentTime)

	gyroXCorr = gyro['x'] - gyroRollCorrection
	push!(gyroRoll, gyroXCorr)
	gyroYCorr = gyro['y'] - gyroPitchCorrection
	push!(gyroPitch, gyroYCorr)
	gyroZCorr = gyro['z'] - gyroYawCorrection
	push!(gyroYaw, gyroZCorr)

	accelXCorr = accel['x'] - accXCorrection
	push!(accX, accelXCorr)
	accelYCorr = accel['y'] - accYCorrection
	push!(accY, accelYCorr)
	accelZCorr = accel['z'] + (9.8005-9.78941028858)
	push!(accZ, accelZCorr)

	magXCorr = mag['x'] 
	push!(magX, magXCorr)
	magYCorr = mag['y'] 
	push!(magY, magYCorr)
	magZCorr = mag['z'] 
	push!(magZ, magZCorr)
	
	sensorValues = [currentTime, gyroXCorr, gyroYCorr, accelXCorr, accelYCorr, accelZCorr, magXCorr, magYCorr, magZCorr]

	append!(dataSet, sensorValues)  # Check whether append or push

	return sensorValues
end

# Kalman Filter

function KalmanFilter(sensorValues, magCorrectionMean, gyroYaw)
	global A
	global B
	global C
	global P
	global Q
	global R
	global state_estimate

	p = deg2rad(sensorValues[1])
	q = deg2rad(sensorValues[2])
	r = deg2rad(sensorValues[3])

	phi_hat_acc = deg2rad(accCalEuler(sensorValues).item((0,0)))
	theta_hat = deg2rad(accCalEuler(sensorValues).item((1,0)))
	psi_hat = deg2rad(accCalEuler(sensorValues).item(2,0)))

	psi_hat_mag= deg2rad(magCalEuler(sensorValues,accCalEuler(sensorValues), magCorrectionMean, gyroYaw).item((2,0)))

	phi_dot = p + sin(phi_hat)*tan(theta_hat)*q + cos(phi_hat)*tan(theta_hat)*r
	theta_dot = cos(phi_hat)*q - sin(phi_hat)*r
	psi_dat = sin(phi_hat)/cos(theta_hat)*q + cos(phi_hat)/cos(theta_hat)*r)
	delta_angle =[[phi_dot], [theta_dot], [psi_dot]]

	# Predict the state
	state_estimate =(A * state_estimate) + (B * delta_angle)

	# Update the state
	Z = [[phi_hat_acc], [theta_hat_acc], [psi_hat_mag - 0.0873]]
	r = Z - C * state_estimate
	S = R + C * P * transpose(C)
	K = p * transpose(C) * inv(S)
	
	state_estimate = state_estimate + K * r

	P = (eye(6) - K * C) * P

	state_estimate_degrees = rad2deg(state_estimate)
	return state_estimate_degrees
end


# Plotting

function graph(figNum, gr1, label1, gr2, label2, gr3, label3, plotTitle, plotYLabel, savePath)
	figure(figNum)
	plot(times, gr1, label = label1)
	plot(times, gr2, label = label2)
	plot(times, gr3, label = label3)
	legend(loc = 'upper right')
	title(plotTitle)
	xlabel("time(ms)")
	ylabel(plotYLabel)
	#  savefig("home/pi/MPU9250/"+FileName+savePath)
end

function savePlot()
	# writedlm("/home/pi/SensorData/"+FileName+/"RawData.csv", dataSet, delimiter = ';')
	graph(1,gyroYaw,'gyroYaw',gyroPitch,'gyroPitch',gyroRoll,'gyroRoll',"Gyro Output vs. Time","Gyro Output (dps)","/rawOutput/gyroOutput.png")
	graph(2,accX,'accX',accY,'Acc Y',accZ,'Acc Z',"Acc. Output vs. Time","Acc. Output (m/s/s)","/rawOutput/accelerometerOutput.png")
	graph(3,magX,'Mag X',magY,'Mag Y',magZ,'Mag Z',"Magnetometer Output vs. Time","Magnetometer Output (uT)","/rawOutput/magnetometerOutput.png")
	graph(4,gyroEulerYaw,'gyroEulerYaw',gyroEulerPitch,'gyroEulerPitch',gyroEulerRoll,'gyroEulerRoll',"Gyro Calculated Angles","Angle (degrees)","/angleEstimations/gyroEulerAngle.png")
	graph(5,accEulerYaw,'accEulerYaw',accEulerPitch,'accEulerPitch',accEulerRoll,'accEulerRoll',"Accelerometer Calculated Angles","Angle (degrees)","/angleEstimations/accEulerAngle.png")
	graph(6,magEulerYaw,'magEulerYaw',magEulerPitch,'magEulerPitch',magEulerRoll,'magEulerRoll',"Magnetometer Calculated Angles","Angle (degrees)","/angleEstimations/magEulerAngle.png")
	graph(7,gyroEulerYaw,'gyroEulerYaw',accEulerYaw,'accEulerYaw',magEulerYaw,'magEulerYaw',"Yaw Angle Comparisons","Angle (degrees)","/comparisons/yawComparisons.png")
	graph(8,gyroEulerPitch,'gyroEulerPitch',accEulerPitch,'accEulerPitch',magEulerPitch,'magEulerPitch',"Pitch Angle Comparisons","Angle (degrees)","/comparisons/pitchComparisons.png")
	graph(9,gyroEulerRoll,'gyroEulerRoll',accEulerRoll,'accEulerRoll',magEulerRoll,'gmagEulerRoll',"Roll Angle Comparisons","Angle (degrees)","/comparisons/rollComparisons.png")
	graph(10,accEulerRoll,'accEulerRoll',filteredRoll,'filteredRoll',gyroEulerRoll,'gyroRoll','Roll Angle Filter Comparison','Angle (degrees)','/comparisons/filteredComparisons/roll.png')
	graph(11,accEulerPitch,'accEulerPitch',filteredPitch,'filteredPitch',gyroEulerPitch,'gyroPitch','Pitch Angle Filter Comparison','Angle (degrees)','/comparisons/filteredComparisons/pitch.png')
	graph(12,magEulerYaw,'magEulerYaw',filteredYaw,'filteredYaw',gyroEulerYaw,'gyroEulerYaw','Yaw Angles Filter Comparison','Anlge (degrees)','/comparisons/filteredComparisons/yaw.png')
	plt.figure(13)
	plt.plot(magX,magY)
end

println("Calibrating the sensor")

for i in 1:100
	calSensorValues = CollectData(0,0,0,0)
	calAccEuler = accCalEuler(calSensorValues)
	calMagEuler = magCalEuler(calSensorValues, calAccEuler, 0, -50)
	append!(magPsiValues, calMagEuler.item((2,0)))
	sleep(0.01)
end
	
magCorrectionMean = mean(magPsiValues)
return magCorrectionMean

gyroRollCorrection = mean(gyroRoll)
gyroPitchCorrection = mean(gyroPitch)
gyroYawCorrection = mean(gyroYaw)

accXCorrection = mean(accX)
accYCorrection = mean(accY)

println("Calibration successful")
sleep(1)

println("Starting the process")

# Resetting the variables

times = []
dataSet = []

gyroYaw = []
gyroPitch = []
gyroRoll = []

accX = []
accY = []
accZ = []

magX = []
magY = []
magZ = []

startTime = time()
currentTime = 0

# Data collection sequence

while currentTime < runTime
	currentTime = time() - startTime
	return currentTime
	
	sensorValues = CollectData(currentTime, gyroRollCorrection, gyroPitchCorrection, gyroYawCorrection)
	
	gyroEulerAngle = gyroCalcEuler(sensorValues, gyroEulerAngle)
	append!(gyroEulerRoll, rad2deg(gyroEulerAngle.item((0,0))))
	append!(gyroEulerPitch, rad2deg(gyroEulerAngle.item((1,0))))
	append!(gyroEulerYaw, rad2deg(gyroEulerAngle.item((2,0))))

	accEulerAngle = accCalEuler(sensorValues)
	append!(accEulerRoll, accEulerAngle.item((0,0)))
	append!(accEulerPitch, accEulerAngle.item((1,0)))
	append!(accEulerYaw, accEulerAngle.item((2,0)))

	magEulerAngle = magCalEuler(sensorValues, accEulerAngle, magCorrectionMean, gyroEulerAngle.item((2,0)))
	append!(magEulerRoll, magEulerAngle.item((0,0)))
	append!(magEulerPitch, magEulerAngle.item((1,0)))
	append!(magEulerYaw, magEulerAngle.item((2,0)))
	
	filteredData = KalmanFilter(sensorValues, magCorrectionMean, rad2deg(gyroEulerAngle.item((2,0))))
	append!(filteredRoll, filteredData.item((0,0)))
	append!(filteredPitch, filteredData.item((1,0)))
	append!(filteredYaw, filteredData.item((2,0)))

	sleep(0.01)

if(saveIndicator == "y" or saveIndicator == "Y")
	println("Saving Data")
	savePlot()
end
