#!/usr/bin/env python
import socket
import time
import pygame

p_pitch = 1.2
i_pitch = 0
d_pitch = 1

p_yaw = 1
i_yaw = 0 
d_yaw = 0

def base10To61(i):
    if i<=9:
        return i+48
    elif i<=9+26:
        return i-10+97
    elif i<=9+26+26:
        return i-36+65

def main():

    global p_pitch
    global i_pitch
    global d_pitch

    global p_yaw
    global i_yaw
    global d_yaw

    address = ('192.168.3.18', 5555)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pygame.init()
    pygame.joystick.init()
    W, H = 320, 240
    screen = pygame.display.set_mode((W, H))
    clock = pygame.time.Clock()
    running = True
    command_to_send=0
    command_last=0
    enable=0
   

    while running:
        joystick_count = pygame.joystick.get_count()
        #print joystick_count

        for i in range(joystick_count):
            joystick = pygame.joystick.Joystick(i)
            joystick.init()
    
            name = joystick.get_name()
            axes = joystick.get_numaxes()

            for i in range( axes ):
                axis = joystick.get_axis( i )
                #print "Axis",i,"value", axis
                if i==0:
                    roll = max(0,min(1000,int(axis*(3.14/3)*500)+500))               
                if i==1:
                    pitch = max(0,min(1000,int(axis*(3.14/3)*500)+500))
                if i==2:
                    yaw = max(0,min(1000,int(axis*(3.14/3)*500)+500))
                if i==3:
                    throttle = int((-axis+1)/2*1000)
            
            buttons = joystick.get_numbuttons()
            for i in range( buttons ):
                button = joystick.get_button( i )
                #print "Button",i, "value: ",button
                if i==4 and button==1:
                    enable=1 
                    string=":2<A1>;"
                    sock.sendto(string,address)
                    time.sleep(2)
                if i==9 and button==1:
                    enable=0
                    string=":2<S1>;"
                    sock.sendto(string,address)
                    time.sleep(2)
                if i==7 and button==1:
                    string=":2<C1>;"
                    sock.sendto(string,address)
                    time.sleep(2)
                if i==6 and button==1:
                    string=":2<R"+chr(base10To61(int(p_pitch*100)/61))+chr(base10To61(int(p_pitch*100)%61))+chr(base10To61(int(i_pitch*100)/61))+chr(base10To61(int(i_pitch*100)%61))+chr(base10To61(int(d_pitch*100)/61))+chr(base10To61(int(d_pitch*100)%61))+chr(base10To61(int(p_yaw*100)/61))+chr(base10To61(int(p_yaw*100)%61))+chr(base10To61(int(i_yaw*100)/61))+chr(base10To61(int(i_yaw*100)%61))+chr(base10To61(int(d_yaw*100)/61))+chr(base10To61(int(d_yaw*100)%61))+">;"
                    sock.sendto(string,address)
                    time.sleep(2)
            
            string=":2<E"+chr(base10To61(pitch/61))+chr(base10To61(pitch%61))+chr(base10To61(roll/61))+chr(base10To61(roll%61))+chr(base10To61(yaw/61))+chr(base10To61(yaw%61))+chr(base10To61(throttle/61))+chr(base10To61(throttle%61))+">;"


            if enable == 1:
                sock.sendto(string,address)

            
         
           

  
            

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False 
         
      
              


        time.sleep(0.3)

    print "Ok."



if __name__ == '__main__':
    main()

