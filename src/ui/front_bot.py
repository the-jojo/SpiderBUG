import pygame

from src.ui.utils import COLOR

RADIUS = 5
SIZE = (RADIUS*2, RADIUS*2)

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

    def render(self):
        pygame.draw.circle(self.image, self.color, (RADIUS, RADIUS), RADIUS)

    def collided(self, other):
        return self != other and self.rect.colliderect(other.rect)

    def set_complete(self):
        self.color = COLOR.GREEN
        self.render()

    def set_dead(self):
        self.color = COLOR.RED
        self.render()

    def update(self):
        self.rect.center = pygame.mouse.get_pos()
        print(pygame.mouse.get_pos())
