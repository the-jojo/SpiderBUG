import pygame, sys, os, math
import pyximport
import numpy as np
from pygame.sprite import Sprite

from src.ui.front_bot import Bot
from src.ui.front_goal import Goal
from src.ui.front_obstacle import Obstacle, SQUARE, RECTANGLE
from src.ui.utils import COLOR

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
pyximport.install(language_level=3, setup_args={"include_dirs": np.get_include()})

pygame.init()

# Set up the drawing window
screen = pygame.display.set_mode([500, 500])

def dist(x1,y1, x2,y2):
    return math.sqrt((x2 - x1) ** 2. + (y2 - y1) ** 2.)


def collided(sprite: Sprite, other: Sprite):
    # Now calculate the offset between the rects.
    offset_x = other.rect.x - sprite.rect.x
    offset_y = other.rect.y - sprite.rect.y
    return sprite.mask.overlap(other.mask, (offset_x, offset_y))



def intersectionPoint( x1,y1, x2,y2, x3,y3, x4,y4 ):
    """ Return the point where the lines through (x1,y1)-(x2,y2)
        and (x3,y3)-(x4,y4) cross.  This may not be on-screen  """
    #Use determinant method, as per
    #Ref: https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    Px = ((((x1*y2)-(y1*x2))*(x3 - x4)) - ((x1-x2)*((x3*y4)-(y3*x4)))) / (((x1-x2)*(y3-y4)) - ((y1-y2)*(x3-x4)))
    Py = ((((x1*y2)-(y1*x2))*(y3 - y4)) - ((y1-y2)*((x3*y4)-(y3*x4)))) / (((x1-x2)*(y3-y4)) - ((y1-y2)*(x3-x4)))
    return Px,Py

def main():
    background = pygame.Surface(screen.get_size())
    background.fill(COLOR.WHITE)
    screen.blit(background, (0,0))

    robot = Bot(None, (25, 250))
    goal = Goal((450, 150))
    obstacle1 = SQUARE(50, (100,100))
    obstacle2 = RECTANGLE(50, 100, (200,100))
    obstacle3 = RECTANGLE(50, 150, (300,100))



    obstacles = pygame.sprite.Group(obstacle1, obstacle2, obstacle3)
    all_sprites = pygame.sprite.Group(*obstacles.sprites(), robot, goal)

    # Run until the user asks to quit
    running = True

    while running:
        # Did the user click the window close button?
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        all_sprites.clear(screen, background)
        all_sprites.update()
        if pygame.sprite.spritecollideany(robot, obstacles, pygame.sprite.collide_mask):
            print("dead")
            robot.set_dead()
        if pygame.sprite.collide_mask(robot, goal):
            print("goal")
            robot.set_complete()

        for obstacle in obstacles.sprites():
            obstacle.set_visible_points(robot.rect.center, obstacles.sprites())


        all_sprites.draw(screen)
        for obstacle in obstacles.sprites():
            obstacle.draw()


        # Flip the display
        pygame.display.flip()

    # Done! Time to quit.
    pygame.quit()

if __name__ == "__main__":
    main()