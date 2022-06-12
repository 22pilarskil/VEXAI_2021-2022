import pygame
import time
import random
import math
from shapely.geometry import Point, Polygon

class Display:
   
	def __init__ (self):	
		pygame.init()
		WIDTH, HEIGHT = 720, 720
		self.WIN = pygame.display.set_mode((WIDTH, HEIGHT))
		field = pygame.image.load("/home/vexai/VEXAI_2021-2022/yolo/sprites/field.png")
		self.field = pygame.transform.scale(field, (WIDTH, HEIGHT))
		mogo = pygame.image.load("/home/vexai/VEXAI_2021-2022/yolo/sprites/mogo.png")
		self.mogo = pygame.transform.scale(mogo, (400, 300))
		ring = pygame.image.load("/home/vexai/VEXAI_2021-2022/yolo/sprites/ring.png")
		self.ring = pygame.transform.scale(ring, (100, 75))
		robot = pygame.image.load("/home/vexai/VEXAI_2021-2022/yolo/sprites/robot.png")
		self.robot = pygame.transform.scale(robot, (60, 60))
		cone = pygame.image.load("/home/vexai/VEXAI_2021-2022/yolo/sprites/cone.png")
		self.cone = pygame.transform.scale(cone, (int(cone.get_width() * 3), int(cone.get_height() * 3)))
		pygame.Surface.set_alpha(self.cone, 30)
		self.boxes = {}
		for box_id in range(1, 37):
			col_num = (box_id + 5) % 6 + 1
			row_num = int((box_id - 1) / 6) + 1
			x1 = (col_num - 1) * 120
			x2 = col_num * 120
			y1 = (row_num - 1) * 120
			y2 = row_num * 120
			self.boxes[box_id] = [Point(x1, y1), Point(x1, y2), Point(x2, y1), Point(x2, y2)]

		print(self.boxes)

	def blit_box(self, box_id, num_mogos, num_rings):
		col_num = (box_id + 5) % 6 + 1
		row_num = int((box_id - 1) / 6) + 1
		if num_mogos != 0:
			mogo_x = (col_num - 1) * 120 + 60
			mogo_y = (row_num - 1) * 120 + 60
			self.WIN.blit(self.mogo, self.center(self.mogo, mogo_x, mogo_y))
			if num_rings != 0:
				ring_y = mogo_y + 50
				base_x = mogo_x - 60
				for i in range(1, num_rings + 1):
					self.WIN.blit(self.ring, self.center(self.ring, base_x + 120 / (num_rings + 1) * i, ring_y))
		elif num_rings != 0:
			ring_y = (row_num - 1) * 120 + 60
			base_x = (col_num - 1) * 120
			for i in range(1, num_rings + 1):
				self.WIN.blit(self.ring, self.center(self.ring, base_x + 120 / (num_rings + 1) * i, ring_y))

	def blit_bot(self, x, y, heading):
		heading += 90
		rotated_bot = pygame.transform.rotate(self.robot, heading)
		rotated_cone = pygame.transform.rotate(self.cone, heading)

		adjusted_x = 590 * math.sin(heading * math.pi/180) + x
		adjusted_y = 590 * math.cos(heading*math.pi/180) + y

		self.WIN.blit(rotated_bot, self.center(rotated_bot, x, y))
		self.WIN.blit(rotated_cone, self.center(rotated_cone, adjusted_x, adjusted_y))


	def center(self, img, x, y):
		return (x - img.get_width() // 2, y - img.get_height() // 2)

	def runner(self, full_map, x, y, heading):
		self.WIN.blit(self.field, (0, 0))
		self.blit_bot(x, y, heading)
		for i in full_map:
			self.blit_box(i, full_map[i][1], full_map[i][0])
		pygame.display.update()
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				quit()

	def endpoint(self, theta, x, y):
		p1 = Point(x, y)
		p2_heading = (theta + 35) * math.pi / 180
		p3_heading = (theta - 35) * math.pi / 180
		p2 = Point(720 * 2 * math.cos(p2_heading) + x, y - 720 * 2 * math.sin(p2_heading))
		p3 = Point(720 * 2 * math.cos(p3_heading) + x, y - 720 * 2 * math.sin(p3_heading))
		coords = [p1, p2, p3]
		print(coords)
                
		return(Polygon(coords))

	def boxes_inside(self, theta, x, y):
		poly = self.endpoint(theta, x, y)
		in_view = {}
		for box_id in self.boxes:
			inside = False
			for point in self.boxes[box_id]:
				if point.within(poly):
					inside = True
			in_view[box_id] = inside
		return in_view

	def det_to_coords(self, x, y, heading, dets):
		new_det = []
		for det in dets:
			dx = x + math.cos((heading - float(det[1])) * math.pi / 180) * float(det[0])
			dy = y + math.sin((heading - float(det[1])) * math.pi / 180) * float(det[0])
			temp = self.robot_coords(dx,dy)
			new_det.append([temp[0], temp[1], det[2]])
		return new_det
	
	def det_to_boxes(self, x, y, heading, dets):
		dets = self.det_to_coords(x, y, heading, dets)
		print("DET COORS: " + str(dets))
		new_det = []
		for det in dets:
			row = math.floor(float(det[1])/120)
			col = math.floor(float(det[0])/120)
			boxnum = 1+col+row*6
			'''
			bx = -1
			for box_id in self.boxes:
				#print("BOUNDS: "+str(self.boxes[box_id]))
				poly = Polygon(self.boxes[box_id])
				point = Point(det[0], det[1])
				#print("POINT: "+str(point))
				if point.within(poly):
					bx = box_id
					break
			if not (bx==-1): new_det.append([bx, det[2]])
			'''
			new_det.append([boxnum,det[2]])
		return new_det

	def robot_coords(self, x, y):
		return [(x+1.8)*720/3.6, (1.8-y)*720/3.6]
				
		
if __name__ == '__main__':
	displayer = Display()
	print(displayer.cone.get_height() / 2)
	while True:
		time.sleep(0.05)
		displayer.runner({1: [1, 0]})
