import cv2
import os 


def generate_path():
	path = os.path.dirname(os.path.abspath(__file__))
	directory_contents = os.listdir(path)
	print(directory_contents)
	highest_iteration = 0
	for item in directory_contents:
		if os.path.isdir(os.path.join(path, item)):
			if int(item[5:]) > highest_iteration:
				highest_iteration = int(item[5:])

	new_path = os.path.join(path, "batch{}".format(highest_iteration + 1))
	os.mkdir(new_path)
	return new_path

def read_video(VIDEO_PATH, FRAME_CAPTURE_RATE):
	new_path = generate_path()
	cap = cv2.VideoCapture(VIDEO_PATH)
	success = cap.grab() 
	print(success)
	fno = 0
	counter = 0
	img_num = 0
	while success:
		if True and counter % FRAME_CAPTURE_RATE == 0:
			_, img = cap.retrieve()
			cv2.imshow("images", img)
			cv2.imwrite("{}/{}.jpg".format(new_path, img_num), img)
			img_num += 1
		counter = counter + 1
		# read next frame
		success = cap.grab()

		if cv2.waitKey(25) & 0xFF == ord('q'):
			break

if __name__ == "__main__":

	VIDEO_PATH = "/Users/michaelpilarski/Desktop/data.mp4"
	FRAME_CAPTURE_RATE = 30

	read_video(VIDEO_PATH, FRAME_CAPTURE_RATE)