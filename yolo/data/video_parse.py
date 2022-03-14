import cv2
import os 
import pathlib
import argparse
import imagehash

from PIL import Image

parser = argparse.ArgumentParser()
parser.add_argument("--file_path", metavar="file_path", type=str)


def read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF, SAVED_HASHES):

	out_dir = "output"

	parent_dir = pathlib.Path(__file__).absolute().parent
	os.chdir(parent_dir)
	if os.path.isdir("{}".format(out_dir)):
		os.system("rm -R {}".format(out_dir))
	os.mkdir(out_dir)

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
			for old_image_hash in hashes:
				if old_image_hash:
					if abs(new_image_hash - old_image_hash) < SIMMILAR_CUTOFF:
						return False
			return True
	
	while success:
		if counter % FRAME_CAPTURE_RATE == 0:
			_, img = cap.retrieve()
			new_image_hash = imagehash.average_hash(Image.fromarray(img))

			if test_image(img, new_image_hash, img_num, hashes):
				hashes[img_num % SAVED_HASHES] = new_image_hash
				cv2.imshow("image", img)
				cv2.imwrite("{}/{}.jpg".format(out_dir, img_num), img)
				img_num += 1
		counter = counter + 1
		success = cap.grab()

		if cv2.waitKey(25) & 0xFF == ord('q'):
			break

if __name__ == "__main__":
	args = parser.parse_args()
	VIDEO_PATH = args.file_path
	FRAME_CAPTURE_RATE = 30
	SIMMILAR_CUTOFF = 5
	SAVED_HASHES = 10

	read_video(VIDEO_PATH, FRAME_CAPTURE_RATE, SIMMILAR_CUTOFF, SAVED_HASHES)
