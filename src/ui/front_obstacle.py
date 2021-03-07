import pygame, numpy
from matplotlib import path

from src.ui.utils import COLOR, MATH


class Obstacle(pygame.sprite.Sprite):
    def __init__(self, size: (int, int), position, corners):
        pygame.sprite.Sprite.__init__(self)
        self.size = size
        self.position = position
        self.corners = corners
        self.marked_boundary = []

        # create transparent background image
        self.image = pygame.Surface( size, pygame.SRCALPHA, 32 )
        self.rect = self.image.get_rect()
        self.rect.center = position

        # Draw a random polygon into the image
        self.draw()

        # Create the collision mask (anything not transparent)
        self.mask = pygame.mask.from_surface( self.image )

        self.sh_polygon = path.Path(self.get_real_corners(), closed=True)
        print('done')

    def get_boundary_points(self):
        N = 50
        return numpy.concatenate((self.sh_polygon.interpolated(N).vertices,
            path.Path([self.get_real_corners()[-1], self.get_real_corners()[0]]).interpolated(N).vertices))

    def get_real_corners(self):
        real_corners = []
        for cx, cy in self.corners:
            real_corners.append((cx + self.rect.topleft[0], cy + self.rect.topleft[1]))
        return real_corners

    def draw(self):
        self.image = 0
        self.image = pygame.Surface( self.size, pygame.SRCALPHA, 32 )
        self.image.set_alpha(255)
        pygame.draw.polygon(self.image, COLOR.BLACK, self.corners)

        for (px, py) in self.marked_boundary:
            pygame.draw.circle(self.image, COLOR.RED, (px-self.rect.topleft[0], py-self.rect.topleft[1]), 4)

    def mark_point(self, x, y):
        self.marked_boundary.append((x,y))

    def reset_boundary(self):
        self.marked_boundary = []

    def is_point_inside(self, x, y):
        return self.sh_polygon.contains_point((x,y), radius=1)

    def set_visible_points(self, robot_pos: (int, int), obstacle_sprites: [pygame.sprite.Sprite]):
        self.reset_boundary()
        boundary_points = self.get_boundary_points()
        for bound_point in boundary_points:
            point_is_clear = True
            line_rob_point = path.Path([robot_pos, bound_point])
            corner_1 = self.get_real_corners()[-1]
            for corner_2 in self.get_real_corners():
                side = path.Path([corner_1, corner_2])
                if (not MATH.linesAreParallel(robot_pos, bound_point, corner_1, corner_2) and
                        not MATH.is_point_btw(corner_1, corner_2, bound_point)):
                    if side.intersects_path(line_rob_point):
                        point_is_clear = False
                        break
                    for o in obstacle_sprites:
                        if o != self:
                            other_corner_1 = o.get_real_corners()[-1]
                            for other_corner_2 in o.get_real_corners():
                                other_side = path.Path([other_corner_1, other_corner_2])
                                if (not MATH.linesAreParallel(robot_pos, bound_point, other_corner_1, other_corner_2)):
                                    if other_side.intersects_path(line_rob_point):
                                        point_is_clear = False
                                        break
                                other_corner_1 = other_corner_2
                corner_1 = corner_2
            if point_is_clear:
                self.mark_point(bound_point[0], bound_point[1])

def SQUARE(size: int, position: (int, int)):
    return Obstacle((size, size), position, [(0,0), (size,0), (size,size), (0,size)])

def TRIANGLE(size: int, position: (int, int)):
    return Obstacle((size, size), position, [(0,0), (size,0), (size,size)])

def RECTANGLE(width: int, length: int, position: (int, int)):
    return Obstacle((width, length), position, [(0,0), (width,0), (width,length), (0,length)])
