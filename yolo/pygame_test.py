import pygame
import time
import random

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


	def center(self, img, x, y):
		return (x - img.get_width() // 2, y - img.get_height() // 2)

	def runner(self, full_map):
		self.WIN.blit(self.field, (0, 0))
		for i in full_map:
			self.blit_box(i, full_map[i][1], full_map[i][0])
		pygame.display.update()
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				quit()
