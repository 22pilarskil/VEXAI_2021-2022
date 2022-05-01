import pygame
import time

pygame.init()


WIDTH, HEIGHT = 720, 720
WIN = pygame.display.set_mode((WIDTH, HEIGHT))
field = pygame.image.load("sprites/field.png")
field = pygame.transform.scale(field, (WIDTH, HEIGHT))
mogo = pygame.image.load("sprites/mogo.png")
mogo = pygame.transform.scale(mogo, (400, 300))
ring = pygame.image.load("sprites/ring.png")
ring = pygame.transform.scale(ring, (100, 75))




def blit_box(box_id, num_mogos, num_rings):
	col_num = (box_id + 5) % 6 + 1
	row_num = int((box_id - 1) / 6) + 1
	if num_mogos != 0:
		mogo_x = (col_num - 1) * 120 + 60
		mogo_y = (row_num - 1) * 120 + 60
		WIN.blit(mogo, center(mogo, mogo_x, mogo_y))
		if num_rings != 0:
			ring_y = mogo_y + 50
			base_x = mogo_x - 60
			for i in range(1, num_rings + 1):
				WIN.blit(ring, center(ring, base_x + 120 / (num_rings + 1) * i, ring_y))
	elif num_rings != 0:
		ring_y = (row_num - 1) * 120 + 60
		base_x = (col_num - 1) * 120
		for i in range(1, num_rings + 1):
			WIN.blit(ring, center(ring, base_x + 120 / (num_rings + 1) * i, ring_y))


def center(img, x, y):
	return (x - img.get_width() // 2, y - img.get_height() // 2)

clock = pygame.time.Clock()
while True:
	WIN.blit(field, (0, 0))
	blit_box(35, 1, 0)
	pygame.display.update()
	for event in pygame.event.get():
		if event.type == pygame.QUIT:
			quit()

	clock.tick(60)


pygame.quit()