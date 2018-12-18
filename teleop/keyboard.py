import pygame
import serial
import time
WIDTH =160
HEIGHT = 120
ser = serial.Serial("/dev/ttyArdM",115200,timeout = 1)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()
loop = True

print("HI")
time.sleep(0.2)
px =0.0
py =0.0
th =0.0

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
            screen.fill((0,0,0))
            time.sleep(0.2)
            
    if keys[pygame.K_ESCAPE]:
        loop = False
    
    odom = ser.readline()
    if  odom.count(',') ==2:
        a,b,c = odom.split(",")
        px = float(a)
        py = float(b)
        th = float(c)
    print(px,py,th)
    
    print (odom)

    ser.flushInput()
    ser.flushOutput()

    time.sleep(0.1)
    ser.write("move {0} {1}\n\r".format((varUp-varDown)*mod,(varLeft-varRight)*mod).encode())
    #print("V {0} {1}\n\r".format(varUp-varDown,varLeft-varRight))
    pygame.draw.circle(screen,(255,0,0),(int(WIDTH/2 - py*0.5),int(HEIGHT/2 -px*0.5)),3,0)
    pygame.display.update()
    pygame.event.pump()  # process event queue
