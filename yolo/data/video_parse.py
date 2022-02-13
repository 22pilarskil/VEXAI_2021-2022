import cv2
import os 
from PIL import Image
import imagehash

def read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF, SAVED_HASHES):
	cap = cv2.VideoCapture(VIDEO_PATH)
	success = cap.grab() 
	hashes = []
	for i in range(SAVED_HASHES):
		hashes.append(None)
	counter = 0
	img_num = 0
	
	def test_image(img, newImageHash, img_num, hashes):
		if len(hashes) == 0:
			return True
		else:
			for oldImageHash in hashes:
				if oldImageHash:
					if abs(newImageHash - oldImageHash) < SIMMILAR_CUTOFF:
						return False
			return True
	
	while success:
		if counter % FRAME_CAPTURE_RATE == 0:
			_, img = cap.retrieve()
			newImageHash = imagehash.average_hash(Image.fromarray(img))

			if test_image(img, newImageHash, img_num, hashes):
				hashes[img_num % SAVED_HASHES] = newImageHash
				cv2.imshow("image", img)
				cv2.imwrite("output/{}.jpg".format(img_num), img)
				img_num += 1
				print(hashes)
		counter = counter + 1
		
		success = cap.grab()

		if cv2.waitKey(25) & 0xFF == ord('q'):
			break

if __name__ == "__main__":
	VIDEO_PATH = "data.mp4"
	FRAME_CAPTURE_RATE = 30
	SIMMILAR_CUTOFF = 5
	SAVED_HASHES = 10

	read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF, SAVED_HASHES)
)
