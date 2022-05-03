import pygame
import time
import random
import math

class Display:
   
	def __init__ (self):	
		pygame.init()
		WIDTH, HEIGHT = 720, 720
		self.WIN = pygame.display.set_mode((WIDTH, HEIGHT))
		field = pygame.image.load("../sprites/field.png")
		self.field = pygame.transform.scale(field, (WIDTH, HEIGHT))
		mogo = pygame.image.load("../sprites/mogo.png")
		self.mogo = pygame.transform.scale(mogo, (400, 300))
		ring = pygame.image.load("../sprites/ring.png")
		self.ring = pygame.transform.scale(ring, (100, 75))
		robot = pygame.image.load("../sprites/robot.png")
		self.robot = pygame.transform.scale(robot, (60, 60))
		cone = pygame.image.load("../sprites/cone.png")
		self.cone = pygame.transform.scale(cone, (int(cone.get_width() * 3), int(cone.get_height() * 3)))
		pygame.Surface.set_alpha(self.cone, 30)

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
		rotated_bot = pygame.transform.rotate(self.robot, heading)
		rotated_cone = pygame.transform.rotate(self.cone, heading)

		adjusted_x = 590 * math.sin(heading * math.pi/180) + x
		adjusted_y = 590 * math.cos(heading*math.pi/180) + y

		self.WIN.blit(rotated_bot, self.center(rotated_bot, x, y))
		self.WIN.blit(rotated_cone, self.center(rotated_cone, adjusted_x, adjusted_y))


	def center(self, img, x, y):
		return (x - img.get_width() // 2, y - img.get_height() // 2)

	def runner(self, full_map):
		self.WIN.blit(self.field, (0, 0))
		self.blit_bot(x=200, y=500, heading=215)
		for i in full_map:
			self.blit_box(i, full_map[i][1], full_map[i][0])
		pygame.display.update()
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				quit()


if __name__ == '__main__':
	displayer = Display()
	print(displayer.cone.get_height() / 2)
	while True:
		time.sleep(0.05)
		displayer.runner({1: [1, 0]})


