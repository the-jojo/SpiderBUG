from typing import Tuple, List

import numpy as np
import pygame

from src.ui.utils import COLOR, MATH
from src.utils.modes import RobMode

RADIUS = 15
SIZE = (RADIUS*2, RADIUS*2)
SPEED = 1.0

class Bot(pygame.sprite.Sprite):
    def __init__(self, position: (int, int) = (25, 25)):
        pygame.sprite.Sprite.__init__(self)

        # create transparent background image
        self.image = pygame.Surface( SIZE, pygame.SRCALPHA, 32 )
        self.rect = self.image.get_rect()
        self.rect.center = position

        # draw robot
        self.color = COLOR.GREY
        self.render()

        # Create the collision mask (anything not transparent)
        self.mask = pygame.mask.from_surface( self.image )

        self.state = RobMode.RUNNING
        self.radius = RADIUS
        self.velocity = (1,1)
        self.center = self.rect.center
        self.past_path = [self.center]

    def render(self):
        self.radius = RADIUS
        pygame.draw.circle(self.image, self.color, (self.radius, self.radius), self.radius)
        if hasattr(self, 'velocity'):
            forward_point = MATH.add(MATH.mult(MATH.to_unit_vector(self.velocity), self.radius-1), (self.radius, self.radius))
            pygame.draw.line(self.image, COLOR.BLACK, (self.radius, self.radius), forward_point, 1)


    def collided(self, other):
        return self != other and self.rect.colliderect(other.rect)

    def set_complete(self):
        self.state = RobMode.GOAL

    def set_dead(self):
        self.state = RobMode.CRASH

    def get_state(self):
        if self.state == RobMode.RUNNING:
            return "Running"
        elif self.state == RobMode.CRASH:
            return "Crash"
        else:
            return "Goal"

    def update(self, goal: Tuple[int, int], visible_obstacles: List[Tuple[int, int]]):
        self.center = np.array(self.center) + (np.array(MATH.to_unit_vector(self.velocity)) * 1)
        self.past_path.append(self.center)
        self.rect.center = self.center
        self.render()

