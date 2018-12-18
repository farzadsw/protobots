import pygame
import serial
import time

ser = serial.Serial("/dev/ttyArdM",115200,timeout = 1)

pygame.init()
screen = pygame.display.set_mode((160, 120))
clock = pygame.time.Clock()
loop = True

print("HI")
time.sleep(0.2)

while loop:
    varLeft = 0
    varRight = 0
    varUp = 0
    varDown = 0
    mod =1
    clock.tick(3)
    keys = pygame.key.get_pressed()
    time.sleep(0.1)
    
    if pygame.key.get_mods() & pygame.KMOD_SHIFT:
            print("shift")
            mod = 2
    
    if keys[pygame.K_a]:

            print("Left !")
            varLeft = 7
            
    if keys[pygame.K_d]:

            print("Right !")
            varRight = 7
            
    if keys[pygame.K_w]:

            print("Up !")
            varUp = 15
            
    if keys[pygame.K_s]:

            print("Down !")
            varDown = 15
            
    if keys[pygame.K_SPACE]:
            print("Stop !")
            varDown =0
            varUp =0
            varRight =0
            varLeft=0
            ser.write("reset\n\r")
            time.sleep(0.2)
            
    if keys[pygame.K_ESCAPE]:
        loop = False
    
    print(ser.readline())

    ser.flushInput()
    ser.flushOutput()

    time.sleep(0.1)
    ser.write("move {0} {1}\n\r".format((varUp-varDown)*mod,(varLeft-varRight)*mod).encode())
    #print("V {0} {1}\n\r".format(varUp-varDown,varLeft-varRight))


    pygame.event.pump()  # process event queue
