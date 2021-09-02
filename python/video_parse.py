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

def read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF):
	new_path = generate_path()
	cap = cv2.VideoCapture(VIDEO_PATH)
	success = cap.grab() 
	print(success)
        hashes = []
	counter = 0
	img_num = 0
	
	def test_image(img, newImageHash, img_num, hashes):
                print('test')
                if len(hashes) == 0:
                        return True  
                else:
                        for oldImageHash in hashes:
                                if abs(newImageHash - oldImageHash) < SIMMILAR_CUTOFF:
                                        return False
                        return True
	
	while success:
		if counter % FRAME_CAPTURE_RATE == 0:
			_, img = cap.retrieve()
                        newImageHash = imagehash.average_hash(Image.fromarray(img))

                        if test_image(img, newImageHash, img_num, hashes):
                                hashes.append(newImageHash)
                                cv2.imshow("image", img)
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
        SIMMILAR_CUTOFF = 5

	read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF)
