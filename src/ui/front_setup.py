import pygame

from src.ui.bot.bot_a_star import Bot_A_Star
from src.ui.front_goal import Goal
from src.ui.front_obstacle import RECTANGLE


class Setup():
	DISPLAY_SIZE = (750, 500)
	ROBOT_START  = (200, 250)
	ROBOT_GOAL   = (600, 250)
	BASE_DIR = ".\\results\\"
	CSV_HEADERS = ['run', 'time', 'final_state']

	@staticmethod
	def course_1():
		obstacle1 = RECTANGLE(10, 150, (200,200))
		obstacle2 = RECTANGLE(10, 150, (350,350))
		obstacle3 = RECTANGLE(10, 150, (600,250))
		return pygame.sprite.Group(obstacle1, obstacle2, obstacle3)

	@staticmethod
	def course_2():
		obstacle1 = RECTANGLE(20, 90, (200,200), (-2, 0))
		#obstacle2 = RECTANGLE(10, 150, (350,350))
		#obstacle3 = RECTANGLE(10, 150, (600,250))
		return pygame.sprite.Group(obstacle1)

	@staticmethod
	def course_3():
		obs = []
		obs.append(RECTANGLE(20, 300, (700, 250), (0, 0)))
		obs.append(RECTANGLE(20, 200, (500, 300), (0, 0)))
		obs.append(RECTANGLE(20, 200, (400, 200), (0, 0)))
		obs.append(RECTANGLE(20, 200, (300, 300), (0, 0)))

		obs.append(RECTANGLE(300, 20, (550, 100), (0, 0)))
		obs.append(RECTANGLE(400, 20, (500, 400), (0, 0)))

		return pygame.sprite.Group(*obs)

	@staticmethod
	def load_a_star(course):
		robot = Bot_A_Star(Setup.ROBOT_START)
		goal = Goal(Setup.ROBOT_GOAL)
		obstacles = course()
		return robot, goal, obstacles

