#!/usr/bin/env python
import rospy
import geometry_msgs.msg
from carry_objects.msg import KeyState
import time
from pygame.locals import *
import pygame
import sys
rospy.init_node('armpose_supporter_keylistener')
pygame.init()
screen = pygame.display.set_mode((400, 330))
pygame.display.set_caption("keyboard event")

pub = rospy.Publisher("keystate", KeyState, queue_size=1)
r = rospy.Rate(100)
msg = KeyState()
msg.rarm = [0, 0, 0, 0, 0, 0, 0, 0]
msg.larm = [0, 0, 0, 0, 0, 0, 0, 0]
msg.mode = 0
while not rospy.is_shutdown():
    screen.fill((0, 0, 0))
    if msg.mode == 1 or msg.mode == 3:
        msg.mode = 0
    for event in pygame.event.get():
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
        if event.type == KEYDOWN:
            if event.key == K_ESCAPE:
                pygame.quit()
                sys.exit()
            elif pygame.key.name(event.key) == "1":
                msg.rarm[0] += 1
            elif pygame.key.name(event.key) == "2":
                msg.rarm[1] += 1
            elif pygame.key.name(event.key) == "3":
                msg.rarm[2] += 1
            elif pygame.key.name(event.key) == "4":
                msg.rarm[3] += 1
            elif pygame.key.name(event.key) == "5":
                msg.rarm[4] += 1
            elif pygame.key.name(event.key) == "6":
                msg.rarm[5] += 1
            elif pygame.key.name(event.key) == "7":
                msg.rarm[6] += 1
            elif pygame.key.name(event.key) == "8":
                msg.rarm[7] += 1
            elif pygame.key.name(event.key) == "q":
                msg.rarm[0] -= 1
            elif pygame.key.name(event.key) == "w":
                msg.rarm[1] -= 1
            elif pygame.key.name(event.key) == "e":
                msg.rarm[2] -= 1
            elif pygame.key.name(event.key) == "r":
                msg.rarm[3] -= 1
            elif pygame.key.name(event.key) == "t":
                msg.rarm[4] -= 1
            elif pygame.key.name(event.key) == "y":
                msg.rarm[5] -= 1
            elif pygame.key.name(event.key) == "u":
                msg.rarm[6] -= 1
            elif pygame.key.name(event.key) == "i":
                msg.rarm[7] -= 1
            elif pygame.key.name(event.key) == "a":
                msg.larm[0] += 1
            elif pygame.key.name(event.key) == "s":
                msg.larm[1] += 1
            elif pygame.key.name(event.key) == "d":
                msg.larm[2] += 1
            elif pygame.key.name(event.key) == "f":
                msg.larm[3] += 1
            elif pygame.key.name(event.key) == "g":
                msg.larm[4] += 1
            elif pygame.key.name(event.key) == "h":
                msg.larm[5] += 1
            elif pygame.key.name(event.key) == "j":
                msg.larm[6] += 1
            elif pygame.key.name(event.key) == "k":
                msg.larm[7] += 1
            elif pygame.key.name(event.key) == "z":
                msg.larm[0] -= 1
            elif pygame.key.name(event.key) == "x":
                msg.larm[1] -= 1
            elif pygame.key.name(event.key) == "c":
                msg.larm[2] -= 1
            elif pygame.key.name(event.key) == "v":
                msg.larm[3] -= 1
            elif pygame.key.name(event.key) == "b":
                msg.larm[4] -= 1
            elif pygame.key.name(event.key) == "n":
                msg.larm[5] -= 1
            elif pygame.key.name(event.key) == "m":
                msg.larm[6] -= 1
            elif pygame.key.name(event.key) == ",":
                msg.larm[7] -= 1
            elif event.key == K_RETURN:
                msg.mode = 1
            elif event.key == K_LSHIFT:
                msg.mode = 2
            elif event.key == K_SPACE:
                msg.mode = 3
        if event.type == KEYUP:
            if pygame.key.name(event.key) == "1":
                msg.rarm[0] -= 1
            elif pygame.key.name(event.key) == "2":
                msg.rarm[1] -= 1
            elif pygame.key.name(event.key) == "3":
                msg.rarm[2] -= 1
            elif pygame.key.name(event.key) == "4":
                msg.rarm[3] -= 1
            elif pygame.key.name(event.key) == "5":
                msg.rarm[4] -= 1
            elif pygame.key.name(event.key) == "6":
                msg.rarm[5] -= 1
            elif pygame.key.name(event.key) == "7":
                msg.rarm[6] -= 1
            elif pygame.key.name(event.key) == "8":
                msg.rarm[7] -= 1
            elif pygame.key.name(event.key) == "q":
                msg.rarm[0] += 1
            elif pygame.key.name(event.key) == "w":
                msg.rarm[1] += 1
            elif pygame.key.name(event.key) == "e":
                msg.rarm[2] += 1
            elif pygame.key.name(event.key) == "r":
                msg.rarm[3] += 1
            elif pygame.key.name(event.key) == "t":
                msg.rarm[4] += 1
            elif pygame.key.name(event.key) == "y":
                msg.rarm[5] += 1
            elif pygame.key.name(event.key) == "u":
                msg.rarm[6] += 1
            elif pygame.key.name(event.key) == "i":
                msg.rarm[7] += 1
            elif pygame.key.name(event.key) == "a":
                msg.larm[0] -= 1
            elif pygame.key.name(event.key) == "s":
                msg.larm[1] -= 1
            elif pygame.key.name(event.key) == "d":
                msg.larm[2] -= 1
            elif pygame.key.name(event.key) == "f":
                msg.larm[3] -= 1
            elif pygame.key.name(event.key) == "g":
                msg.larm[4] -= 1
            elif pygame.key.name(event.key) == "h":
                msg.larm[5] -= 1
            elif pygame.key.name(event.key) == "j":
                msg.larm[6] -= 1
            elif pygame.key.name(event.key) == "k":
                msg.larm[7] -= 1
            elif pygame.key.name(event.key) == "z":
                msg.larm[0] += 1
            elif pygame.key.name(event.key) == "x":
                msg.larm[1] += 1
            elif pygame.key.name(event.key) == "c":
                msg.larm[2] += 1
            elif pygame.key.name(event.key) == "v":
                msg.larm[3] += 1
            elif pygame.key.name(event.key) == "b":
                msg.larm[4] += 1
            elif pygame.key.name(event.key) == "n":
                msg.larm[5] += 1
            elif pygame.key.name(event.key) == "m":
                msg.larm[6] += 1
            elif pygame.key.name(event.key) == ",":
                msg.larm[7] += 1
            elif event.key == K_LSHIFT:
                msg.mode = 0
    pygame.display.update()
    pub.publish(msg)
    r.sleep()
