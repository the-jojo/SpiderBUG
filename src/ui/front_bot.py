import pygame, math
import numpy as np

from src.ui.utils import COLOR, MATH

RADIUS = 15
SIZE = (RADIUS*2, RADIUS*2)
SPEED = 1.0

class Bot(pygame.sprite.Sprite):
    def __init__(self, back_robot, position: (int, int) = (25, 25)):
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

        self.back_robot = back_robot
        self.velocity = (1,1)
        self.center = self.rect.center
        self.past_path = [self.center]

    def render(self):
        pygame.draw.circle(self.image, self.color, (RADIUS, RADIUS), RADIUS)
        if hasattr(self, 'velocity'):
            forward_point = MATH.add(MATH.mult(MATH.to_unit_vector(self.velocity), RADIUS-1), (RADIUS, RADIUS))
            pygame.draw.line(self.image, COLOR.BLACK, (RADIUS, RADIUS), forward_point, 1)


    def collided(self, other):
        return self != other and self.rect.colliderect(other.rect)

    def set_complete(self):
        self.color = COLOR.GREEN
        self.render()

    def set_dead(self):
        self.color = COLOR.RED
        self.render()

    def update(self):
        self.velocity = MATH.add(self.velocity, (0, -0.01))
        self.center = np.array(self.center) + (np.array(MATH.to_unit_vector(self.velocity)) * 1)
        self.past_path.append(self.center)
        self.rect.center = self.center
        self.render()

