import numpy as np
import os
import pygame
import pyximport
import sys
import time

from src.ui.front_setup import Setup
from src.ui.utils import COLOR, IO
from src.utils.modes import ExMode

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
pyximport.install(language_level=3, setup_args={"include_dirs": np.get_include()})

pygame.init()


def main():
    # Set up the drawing window
    screen = pygame.display.set_mode(Setup.DISPLAY_SIZE)

    iter_n = 1
    path_n = 0
    path_t = 4

    print("Starting simulation...")
    print("Controls: [space] to pause/resume")
    print("          [w]     to print the mouse (x,y) position")
    print("          [q]     to quit the simulation")
    print("          [s]     to skip the iteration")

    # Run until the user asks to quit
    ex_mode = ExMode.RUNNING

    while path_n < path_t and ex_mode != ExMode.QUIT:
        print("Iteration " + str(path_n))

        # Set up the background
        background = pygame.Surface(screen.get_size())
        background.fill(COLOR.WHITE)
        screen.blit(background, (0,0))

        # get the robot, goal and obstacles
        robot, goal, obstacles = Setup.load_a_star(Setup.course_3)

        all_sprites = pygame.sprite.Group(*obstacles.sprites(), robot, goal)

        IO.write_headers_to_csv_if_needed(Setup.BASE_DIR, robot.__class__.__name__, iter_n, Setup.CSV_HEADERS)

        ex_mode = ExMode.RUNNING

        time_0 = time.perf_counter()
        while ex_mode != ExMode.STOP and ex_mode != ExMode.QUIT:
            # Handle Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    ex_mode = ExMode.QUIT
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_w:
                        print(pygame.mouse.get_pos())
                    if event.key == pygame.K_q:
                        ex_mode = ExMode.QUIT
                    if event.key == pygame.K_s:
                        ex_mode = ExMode.STOP
                    if event.key == pygame.K_SPACE:
                        if ex_mode == ExMode.PAUSED:
                            ex_mode = ExMode.RUNNING
                        elif ex_mode == ExMode.RUNNING:
                            ex_mode = ExMode.PAUSED

            if ex_mode == ex_mode.RUNNING:
                all_sprites.clear(screen, background)

                all_visible_points = []
                for obstacle in obstacles.sprites():
                    obstacle.set_visible_points(robot.rect.center, obstacles.sprites())
                    all_visible_points = all_visible_points + obstacle.marked_boundary

                robot.update(goal.rect.center, all_visible_points)

                if pygame.sprite.spritecollideany(robot, obstacles, pygame.sprite.collide_mask):
                    print("Robot crashed")
                    robot.set_dead()
                    ex_mode = ExMode.STOP

                if pygame.sprite.collide_mask(robot, goal):
                    print("Robot reached goal")
                    robot.set_complete()
                    ex_mode = ExMode.STOP

                all_sprites.draw(screen)

                for obstacle in obstacles.sprites():
                    obstacle.update()
                    obstacle.draw()

                # draw past path of robot
                p1 = robot.past_path[0]
                for p2 in robot.past_path[1:]:
                    pygame.draw.line(background, COLOR.RED, p1, p2, 2)
                    p1 = p2

            # Flip the display
            pygame.display.flip()

        # done iteration. Robot either crashed or reached the goal
        time_1 = time.perf_counter()

        # write path to file
        IO.write_path(Setup.BASE_DIR, robot.__class__.__name__, iter_n, path_n, robot.past_path)

        # write csv to file
        IO.write_info(Setup.BASE_DIR, robot.__class__.__name__, iter_n, path_n, time_1 - time_0, robot.get_state())

        # increment path identifier
        path_n += 1

    # Done! Time to quit.
    print("Goodbye...")
    pygame.quit()

if __name__ == "__main__":
    main()