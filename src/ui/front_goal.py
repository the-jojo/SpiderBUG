import pygame

from src.ui.utils import COLOR

RADIUS = 15
SIZE = (RADIUS*2, RADIUS*2)

class Goal(pygame.sprite.Sprite):
    def __init__(self, position: (int, int) = (25, 25)):
        pygame.sprite.Sprite.__init__(self)

        # create transparent background image
        self.image = pygame.Surface( SIZE, pygame.SRCALPHA, 32 )
        self.rect = self.image.get_rect()
        self.rect.center = position

        pygame.draw.circle(self.image, COLOR.LIGHT_GREEN, (RADIUS,RADIUS), RADIUS)

        # Create the collision mask (anything not transparent)
        self.mask = pygame.mask.from_surface( self.image )


    def update(self):
        pass
        #self.rect.center = pygame.mouse.get_pos()
