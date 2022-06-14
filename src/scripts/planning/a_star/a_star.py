#!/usr/bin/env python
from matplotlib.pyplot import grid
import rospy
import csv
import numpy as np
from queue import PriorityQueue
import pygame 



class Plot:
    def __init__(self):
        # self.map = map
        # self.hight = len(self.map)
        # self.width = len( self.map[0])
        self.width = 1200
        self.height = 800
        self.rows = 100
        self.col = 100
        self.frame = pygame.display.set_mode((self.width, self.height))
        self.color = WHITE 

    def draw_grid(self):
        h_gap = self.height // self.rows
        v_gap = self.width // self.col
        # gap = self.width // self.rows
        for i in range(self.rows):
            pygame.draw.line(self.frame, GREY, (0, i * v_gap), (self.height, i * v_gap))
            for j in range(self.rows):
                pygame.draw.line(self.frame, GREY, (j * h_gap, 0), (j * h_gap, self.width))
            
        # pygame.draw.line(self.frame, GREY, (h_gap, 0), (h_gap, self.height) )
        # pygame.draw.line(self.frame, GREY, (0, v_gap), (v_gap, self.height) )


    def draw(self):
        self.frame.fill(WHITE)
        # for row in grid:
        #     for spot in row:
        #         spot.draw(self.frame)
        
        self.draw_grid()
        pygame.display.update()




if __name__ == '__main__':
    GREY = (128, 128, 128)
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    run = True
    p = Plot()
    while run:
        p.draw()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
    