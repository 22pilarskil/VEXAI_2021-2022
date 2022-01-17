import cv2

cap = cv2.VideoCapture('video_data.mp4')

ret, first_frame = cap.read()
first_frame_gray = cv2.cvtColor(first_frame, cv2.COLOR_BGR2GRAY)
first_frame_histogram = cv2.calcHist([first_frame_gray], [0], None, [256], [0, 256])

frame_number = 1
while (cap.isOpened()):
    ret, frame = cap.read()

    second_frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    second_frame_histogram = cv2.calcHist([second_frame_gray], [0], None, [256], [0, 256])
    distance = 0

    i = 0
    while i<len(first_frame_histogram) and i<len(second_frame_histogram):
        distance += (first_frame_histogram[i] - second_frame_histogram[i]) ** 2
        i += 1
    distance = distance ** (1/2)

    if distance > 15000.0:
        frame_number += 1
        cv2.imwrite("Frame {}.jpg".format(frame_number), frame)
        print("Frame {}.jpg".format(frame_number))

    first_frame_histogram = second_frame_histogram
