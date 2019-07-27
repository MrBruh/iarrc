# USAGE
# python openvino_real_time_object_detection.py --prototxt MobileNetSSD_deploy.prototxt --model MobileNetSSD_deploy.caffemodel

# import the necessary packages
from imutils.video import VideoStream
from imutils.video import FPS
from picamera import PiCamera
import numpy as np
import argparse
import imutils
import time
import cv2
import math
import serial
import argparse
import time
import random



vs = VideoStream(usePiCamera=True).start()
time.sleep(2.0)
vs.camera.exposure_mode = 'off'
vs.camera.awb_mode = 'off'
vs.camera.awb_gains = (1.5, 1.5)
vs.camera.framerate = 16
fps = FPS().start()

def nothing(x):
	pass

def writemotor(value):
	ser.write(b'f')
	ser.write(str(value).encode('utf-8'))
	ser.write(b'\n')

def writebrake():
	ser.write(b'b')

def writesteer(value):
	value = int(value)
	ser.write(b'g')
	if value > 0:
		ser.write(str(value).encode('utf-8'))
		ser.write(b'\n')
	elif value < 0:
		value *= -1
		ser.write(str(value).encode('utf-8'))
		ser.write(b'-')
		ser.write(b'\n')
	elif value == 0:
		ser.write(b'0\n')
	else:
		return

def writesteerangle(value):
	value = int(value)
	print("anglesteer: ", value)
	ser.write(b'v')
	if value > 0:
		ser.write(str(value).encode('utf-8'))
		ser.write(b'\n')
	elif value < 0:
		value *= -1
		ser.write(str(value).encode('utf-8'))
		ser.write(b'-')
		ser.write(b'\n')
	elif value == 0:
		ser.write(b'0\n')
	else:
		return

def writecurrentsteer():
	ser.write(b'c')

def writereset():
	ser.write(b'r')

def filter(frame, params):
	lower = np.array([params[1],params[3],params[5]])
	upper = np.array([params[0],params[2],params[4]])

	mask = cv2.inRange(frame, lower, upper)
	#res = cv2.bitwise_and(frame,frame, mask= mask)
	return mask

def detect_lines(smoothed, low_threshold, high_threshold, is_yellow):
	edges = cv2.Canny(smoothed, low_threshold, high_threshold)

	rho = 1  # distance resolution in pixels of the Hough grid
	theta = np.pi / 180  # angular resolution in radians of the Hough grid
	threshold = 15  # minimum number of votes (intersections in Hough grid cell)
	min_line_length = 50  # minimum number of pixels making up a line
	max_line_gap = 20  # maximum gap in pixels between connectable line segments
	
	# Run Hough on edge detected image
	# Output "lines" is an array containing endpoints of detected line segments
	lines = cv2.HoughLinesP(edges, rho, theta, threshold, np.array([]),
						min_line_length, max_line_gap)
	return lines

def draw_slope(lines, line_image, y_threshold, min_length, is_yellow):
	weighted_slopes = []
	avg_slope = 0
	distance = 0
	avg_x = 0
	avg_y = 0
	total_distance = 0
	total_valid_lines = 0
	if lines is not None:
		for line in lines:
			for x1,y1,x2,y2 in line:
				if y1 > y_threshold:
					cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
					distance = ((x2-x1)**2+(y2-y1)**2)**0.5
					if is_yellow:
						print("points: ", x1,y1,x2,y2, distance, "slope: ", (y2-y1)/(x2-x1), math.atan2((y2-y1),(x2-x1))*57)
					if(distance > min_length):
						if x1 == x2:
							weighted_slopes.append(distance*math.pi/2)
							print("infinity")
						else:
							weighted_slopes.append(distance*math.atan2((y2-y1),(x2-x1)))
						total_valid_lines = total_valid_lines + 1
						total_distance = total_distance + distance
						avg_x += (x1+x2)/2
						avg_y += (y1+y2)/2
				elif y2 > y_threshold:
					cv2.line(line_image,(x1,y1),(x2,y2),(255,0,0),5)
					distance = ((x2-x1)**2+(y2-y1)**2)**0.5
					if is_yellow:
						print("points: ", x1,y1,x2,y2, distance, "slope: ", (y2-y1)/(x2-x1), math.atan2((y2-y1),(x2-x1))*57)
					if(distance > min_length):
						if x1 == x2:
							weighted_slopes.append(distance*math.pi/2)
							print("infinity")
						else:
							weighted_slopes.append(distance*math.atan2((y2-y1),(x2-x1)))
						total_valid_lines = total_valid_lines + 1
						total_distance = total_distance + distance
						avg_x += (x1+x2)/2
						avg_y += (y1+y2)/2
	
	avg_slope_rad = 0
	if total_valid_lines != 0:
		for ws in weighted_slopes:
			avg_slope_rad += ws
		avg_slope_rad = avg_slope_rad / total_distance
		avg_slope = math.tan(avg_slope_rad) 

		avg_x = avg_x/total_valid_lines
		avg_y = avg_y/total_valid_lines

		#b = avg_slope*200
		if avg_slope == 0.0:
			print("horizontal")
			print(avg_x, avg_slope, total_valid_lines)
			cv2.line(line_image, (int(avg_x), 0), (int(avg_x), 400),(0,0,255),5 )
		elif avg_slope == float("inf"):
			print("vertical")
			print(avg_y, avg_slope, total_valid_lines)
			cv2.line(line_image, (0, int(avg_y)), (400, int(avg_y)),(0,0,255),5 )
		else:
			b = avg_y - avg_slope*avg_x
			if(b > 1000000):
				b= 1000000
			if is_yellow:
				print("b:", b, "slope:", avg_slope, "total:", total_valid_lines, "avg_x:", avg_x, "avg_y:", avg_x, "avg_slope_rad:", avg_slope_rad)
			#print(int((0-b)/avg_slope),0)
			#print(int((400-b)/avg_slope),400)
			cv2.line(line_image,(int((0-b)/avg_slope),0),(int((400-b)/avg_slope),400),(0,0,255),5)
			#print("b:", b, "slope:", avg_slope, "total:", total_valid_lines, math.atan(avg_slope)*51)
			#cv2.line(line_image, (200,400), (int(b), 0),(0,0,255),5)
	return line_image, avg_slope

def detect_start(blur, params):
	red = filter(blur, params)
	pixels = cv2.countNonZero(red)
	print(pixels)
	if pixels > 350:
		return True
	else:
		return False

def detect_end(blur, params):
	magenta = filter(blur, params)
	lines = detect_lines(magenta, 50, 200, False)
	if lines is not None:
		for line in lines:
			print(line)
			for x1,y1,x2,y2 in line:
				if y1 > 250:
					print("ended", y1)
					return True
				elif y2 > 250:
					print("ended", y2)
					return True
				else:
					return False
	return False

def horizontal_detect(lines):
	detected = False
	if lines is not None:
		for line in lines:
			for x1,y1,x2,y2 in line:
				if y1 > 400:
					detected = caluculate_horizontal(x1,y1,x2,y2)
				elif y2 > 400:
					detected = caluculate_horizontal(x1,y1,x2,y2)
				else:
					pass
		if detected:
			return True
	return False

def caluculate_horizontal(x1, y1, x2, y2):
	slope = (y2-y1)/(x2-x1)
	avg_y = (y1+y2)/2
	avg_x = (x1+x2)/2
	b = avg_y - slope*avg_x
	point = slope*avg_x + b
	if point < 450 and point > 400:
		return True
	return False

def assign_trackbar(window):
	Blur = cv2.getTrackbarPos("Blur", window)
	if Blur % 2 == 0:
		Blur += 1

	HMax = cv2.getTrackbarPos("HMax", window)
	HMin = cv2.getTrackbarPos("HMin", window)

	SMax = cv2.getTrackbarPos("SMax", window)
	SMin = cv2.getTrackbarPos("SMin", window)

	VMax = cv2.getTrackbarPos("VMax", window)
	VMin = cv2.getTrackbarPos("VMin", window)
	params = [HMax, HMin, SMax, SMin, VMax, VMin, Blur]
	return params


cv2.namedWindow('Yellow')

cv2.createTrackbar("HMax", "Yellow",255,255,nothing)
cv2.createTrackbar("HMin", "Yellow",124,255,nothing)

cv2.createTrackbar("SMax", "Yellow",255,255,nothing)
cv2.createTrackbar("SMin", "Yellow",81,255,nothing)

cv2.createTrackbar("VMax", "Yellow",255,255,nothing)
cv2.createTrackbar("VMin", "Yellow",191,255,nothing)

cv2.createTrackbar("Blur", "Yellow",5,20,nothing)


cv2.namedWindow('White')

cv2.createTrackbar("HMax", "White",167,255,nothing)
cv2.createTrackbar("HMin", "White",135,255,nothing)

cv2.createTrackbar("SMax", "White",52,255,nothing)
cv2.createTrackbar("SMin", "White",0,255,nothing)

cv2.createTrackbar("VMax", "White",255,255,nothing)
cv2.createTrackbar("VMin", "White",190,255,nothing)

cv2.createTrackbar("Blur", "White",0,20,nothing)

cv2.namedWindow('Red')

cv2.createTrackbar("HMax", "Red",255,255,nothing)
cv2.createTrackbar("HMin", "Red",0,255,nothing)

cv2.createTrackbar("SMax", "Red",255,255,nothing)
cv2.createTrackbar("SMin", "Red",0,255,nothing)

cv2.createTrackbar("VMax", "Red",255,255,nothing)
cv2.createTrackbar("VMin", "Red",0,255,nothing)

cv2.createTrackbar("Blur", "Red",5,20,nothing)

cv2.namedWindow('Magenta')

cv2.createTrackbar("HMax", "Magenta",167,255,nothing)
cv2.createTrackbar("HMin", "Magenta",135,255,nothing)

cv2.createTrackbar("SMax", "Magenta",145,255,nothing)
cv2.createTrackbar("SMin", "Magenta",86,255,nothing)

cv2.createTrackbar("VMax", "Magenta",255,255,nothing)
cv2.createTrackbar("VMin", "Magenta",191,255,nothing)

cv2.createTrackbar("Blur", "Magenta",5,20,nothing)

cv2.namedWindow('Green')

cv2.createTrackbar("Line_Threshold", "Green",200,400,nothing)
cv2.createTrackbar("Min_Threshold", "Green",81,200,nothing)
cv2.createTrackbar("Max_Threshold", "Green",150,200,nothing)
cv2.createTrackbar("Min_Line_Threshold", "Green", 50, 400, nothing)
cv2.createTrackbar("Slope_add", "Green", 0, 30, nothing)
cv2.createTrackbar("Slope_sub", "Green", 0, 30, nothing)
cv2.createTrackbar("Gaussian", "Green", 7, 15, nothing)

cv2.namedWindow('awb')
cv2.createTrackbar("bg1", "awb", 15, 80, nothing)
cv2.createTrackbar("rg1", "awb", 12, 80, nothing)
cv2.createTrackbar("iso", "awb", 800, 800, nothing)
cv2.createTrackbar("comp", "awb", 25, 50, nothing)

#arguments for # of lanes to follow
parser = argparse.ArgumentParser()
parser.add_argument("-l", "--lanes", type=int, default=0, choices=range(0, 3),
                    help="The number of games to simulate")
args = parser.parse_args()
lane_follow_option = args.lanes
#print(lane_follow_option)	

steer_angle = 9
ser = serial.Serial('/dev/ttyS0', 9600)
writecurrentsteer()
time.sleep(2.0)
white_offset = 0
yellow_offset = 0
offset_count = 0
offset_added = False
#writesteer(steer_angle)
#frame = cv2.imread('1.jpg')
s = 450
#frame = cv2.resize(frame, (s,s), 0, 0, cv2.INTER_AREA)
#hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#blur = cv2.GaussianBlur(hsv,(7,7),0)

frame_count = 0
# loop over the frames from the video stream
print("succeeded")
print(vs.camera.resolution)
race_almost_started = False
race_primed = False
race_started = False
race_started_count = 0
race_ended = False
motor_started = False
motor_braked = False
direction_chosen = False
direction_count = 0
while True:   

	bg = (cv2.getTrackbarPos("bg1", "awb")/10)#, cv2.getTrackbarPos("bg2", "awb")) 
	rg = (cv2.getTrackbarPos("rg1", "awb")/10)#, cv2.getTrackbarPos("rg2", "awb"))
	vs.camera.awb_gains = (rg, bg)
	iso = cv2.getTrackbarPos("iso", "awb")
	vs.camera.iso = iso
	exposure = cv2.getTrackbarPos("comp", "awb") - 25
	vs.camera.exposure_compensation = exposure
	guassian = cv2.getTrackbarPos("Gaussian", "Green")
	if guassian % 2 == 0:
		guassian += 1
	frame = vs.read()
	#frame = cv2.imread('1.jpg')
	frame = cv2.resize(frame, (s,s), 0, 0, cv2.INTER_AREA)
	lines_edges = frame 
	hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
	blur = cv2.GaussianBlur(hsv,(guassian,guassian),0)

	red_params = assign_trackbar("Red")
	magenta_params = assign_trackbar("Magenta")
	white_params = assign_trackbar("White")
	if(lane_follow_option == 2):
		yellow_params = assign_trackbar("Yellow")

	white = filter(blur, white_params)
	smoothed_white = cv2.medianBlur(white,white_params[6])
	if lane_follow_option == 2:
		yellow = filter(blur, yellow_params)
		smoothed_yellow = cv2.medianBlur(yellow,yellow_params[6])

	if race_started and not race_ended:
		if not motor_started:
			writemotor(2)
			motor_started = True
		print("starting again", race_started, race_ended)
		
		low_threshold = cv2.getTrackbarPos("Min_Threshold", "Green")
		high_threshold = cv2.getTrackbarPos("Max_Threshold", "Green")
		line_image = np.copy(frame) * 0  # creating a blank to draw lines on
		y_threshold = cv2.getTrackbarPos("Line_Threshold", "Green")
		min_line = cv2.getTrackbarPos("Min_Line_Threshold", "Green")

		lines = detect_lines(smoothed_white, low_threshold, high_threshold, False)
		white_tup = draw_slope(lines, line_image, y_threshold, min_line, False)
		line_image = white_tup[0]

		if lane_follow_option == 2:
			lines = detect_lines(smoothed_yellow, low_threshold, high_threshold, False)
			yellow_tup = draw_slope(lines, line_image, y_threshold, min_line, False)
			line_image = yellow_tup[0]
				
		cv2.line(line_image,(0,y_threshold),(400,y_threshold),(0,255,0),5)
		lines_edges = cv2.addWeighted(frame, 0.8, line_image, 1, 0)

		white_angle = -1*(math.atan(white_tup[1])*180/math.pi)
		if lane_follow_option == 2:
			yellow_angle = -1*(math.atan(yellow_tup[1])*180/math.pi)
			avg_slope = white_angle + yellow_angle
		#print("avg_slope & writepos:",  avg_slope, white_angle, yellow_angle)

		if lane_follow_option == 2:
			if offset_count >= 20 and not offset_added:
				yellow_offset = yellow_angle
				white_offset = white_angle
				print("added", white_angle, white_offset, yellow_offset, yellow_offset)
				offset_added = True
			elif offset_count < 20:
				offset_count += 1
				print( offset_count)
			else:
				pass

		if lane_follow_option == 1:
			if not direction_chosen:
				if horizontal_detect(lines):
					choice = random.randint(0, 3)
					if choice == 0:
						writesteerangle(0)
					elif choice == 1:
						writesteerangle(90)
					elif choice == 2:
						writesteerangle(-90)
					direction_chosen = True
			else:
				direction_count += 1
				if direction_count >= 30:
					direction_chosen = False
					direction_count = 0

		if frame_count >= 2:
			if(lane_follow_option == 2):
				positive = cv2.getTrackbarPos("Slope_add", "Green")
				negative = cv2.getTrackbarPos("Slope_sub", "Green")
				writepos = avg_slope + positive - negative	
				#print("unrounded: ", writepos)
				if white_angle != 0 and yellow_angle != 0:
					print(white_angle, yellow_angle, avg_slope)
					writesteerangle(writepos)
				elif white_angle == 0:
					print(yellow_offset, yellow_angle, avg_slope)
					writesteerangle(avg_slope + yellow_offset)
				elif yellow_angle == 0:
					print(white_offset, white_angle, avg_slope)
					writesteerangle(avg_slope + white_offset)
				else:
					pass
			elif(lane_follow_option == 1):
				positive = cv2.getTrackbarPos("Slope_add", "Green")
				negative = cv2.getTrackbarPos("Slope_sub", "Green")
				writepos = 10*white_angle + 10*positive - 10*negative
				print(writepos)
				writesteer(writepos)
			elif(lane_follow_option == 0):
				writereset()
			frame_count = 0
		else:
			#print(frame_count)
			frame_count += 1
		#race_ended = detect_end(blur, magenta_params)
	else:
		print("getting ready", race_started, race_ended, race_started_count, race_primed, race_almost_started)
		race_almost_started = detect_start(blur, red_params)
		if race_almost_started == True:
			race_started_count += 1
		else:
			race_started_count = 0
		if race_started_count >= 20:
			race_primed = True
		if race_primed == True and not race_almost_started:
			race_started = True
	if race_ended and not motor_braked:
		writemotor(20)
		time.sleep(1)
		writebrake()
		motor_braked = True

	# show the output frame
	
	cv2.imshow("White", smoothed_white)
	if lane_follow_option == 2:
		cv2.imshow("Yellow", smoothed_yellow)
	cv2.imshow("Main", lines_edges)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

	# update the FPS counter
	fps.update()
	#time.sleep(0.02)

# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
