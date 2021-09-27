#!/usr/bin/env python

import rospy
import math
import time
import sys, select, termios, tty


from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

x = 0
y = 0
z = 0

from Tkinter import *
import ttk
import threading 
import ars_control

global window

 
window= Tk()
window.config(background= "#41B77F")

prompt='Click any button, or press a key' 
L = Label(window, text=prompt, width=len(prompt)) 
L.pack() 

def key(event): 
    if event.char == event.keysym: 
       msg ='Normal Key %r' % event.char 
    elif len(event.char) == 1: 
       msg ='Punctuation Key %r (%r)' % (event.keysym, event.char) 
    else: 
       msg ='Special Key %r' % event.keysym 
       L.config(text=msg) 
L.bind_all('<Key>', key) 

def do_mouse(eventname): 
    def mouse_binding(event): 
       msg = 'Mouse event %s' % eventname 
       L.config(text=msg) 
    L.bind_all('<%s>'%eventname, mouse_binding) 

for i in range(1,4): 
    do_mouse('Button-%s' % i) 
    do_mouse('ButtonRelease-%s' % i) 
    do_mouse('Double-Button-%s' % i) 


def quit():
             
    distro = window.destroy()
    exit = sys.exit()  
    return distro, exit

   

def getdistance():
    distance = float(distance_selected.get())
    return distance

def getspeed():
    speed = float(speed_selected.get())
    return speed

def init_state():
    speed= 0.0
    distance= 0.0
    x = 0
    y = 0
    z = 0
    yaw = 0
    t = threading.Thread(target= ars_control.home())
    t.start()


def moveforward_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveX(speed, distance, True))
    t.start()

def moveback_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveX(speed, distance, False))
    t.start()

def moveleft_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveY(speed, distance, True))
    t.start()

def moveright_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveY(speed, distance, False))
    t.start()

def return_home():
    t = threading.Thread(target= ars_control.control())
    t.start()

def goup_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveZ(speed, distance, True))
    t.start()
def godown_background():
    speed= getspeed()
    distance=getdistance()
    t = threading.Thread(target= ars_control.moveZ(speed, distance, False))
    t.start()

def rotationmoveleft_background():
    t = threading.Thread(target= ars_control.rotate(10, 65, True))
    t.start()    

def rotationmoveright_background():
    t = threading.Thread(target= ars_control.rotate(10, 65, False))
    t.start()  

#Define a callback function for exit
def quit_program(e):
   window.destroy()

if __name__ == '__main__':


    try:
        
        rospy.init_node('ars_remote_controller', anonymous=True)
        

        position_topic = "/robot_base_link"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, ars_control.Odometry_Callback) 
        time.sleep(2)

        label_title= Label(window, text= "Controller", font=("Courrier",40), bg = "#41B77F", fg= "white")
        label_title.pack()

        window.title("Nao_Drone")
        window.geometry("1080x600")
        window.minsize(1000,500)


        #user choice of speed and distance
        speed_label= Label(window,text = "Speed").place(x=36,y=50)
        distance_label= Label(window, text= "Distance").place(x=740,y=50)
        
        distance_var= StringVar()
        distance_selected= ttk.Combobox(window, width=20,textvariable=distance_var)
        distance_selected['values']=('0','0.000001','0.000002','0.000003','0.000005')
        distance_selected.place(x=800,y=50)
        distance_selected.current(0)
        
        speed_var= StringVar()
        speed_selected= ttk.Combobox(window, width=20,textvariable=speed_var)
        speed_selected['values']=('0.0','0.1','0.2','0.3','0.5')
        speed_selected.place(x=80,y=50)
        speed_selected.current(0)
       
        
        moveforward_button = Button(window, text="Move Forward", height= "3", width="20",command = moveforward_background).place(x=450,y=150)
        moveback_button= Button(window, text="Move Back", height= "3", width="20", command = moveback_background).place(x=450,y=350)
        moveleft_button= Button(window, text="Move Left", height= "3", width="20", command= moveleft_background).place(x=350,y=250)
        moveright_button = Button(window, text="Move Right", height= "3", width="20", command = moveright_background).place(x=550,y=250)
        goup_button = Button(window, text= "Go Up", height="3", width="20", command= goup_background).place(x = 450, y = 450)
        godown_button = Button(window, text= "Go Down", height="3", width= "20", command= godown_background).place(x= 450, y =520)

        
        quit_button = Button(window, text= "Quit Interface", height = "3", width= "20", command = quit).place(x=30, y= 300)
        init_state_button = Button(window, text= "Initial State", height = "3", width= "20", command = init_state).place(x=30, y= 420)    
        
        rotationmoveleft_button= Button(window, text= "Rotate to the left", height="3", width= "20", command= rotationmoveleft_background).place(x=800, y=450)
        rotationmoveright_button = Button(window, text = "Rotate to the right", height="3", width= "20", command= rotationmoveright_background).place(x=800,y=520)
        
        #Add a Label widget
        label = Label(window, text= "Press Ctrl + x to Exit", font= ('Helvetica 15 bold'))
        label.pack(pady=10)

        #Bind the Keyboard shortcut Key
        window.bind('<Control-x>', quit_program)
        window.mainloop() 


        while True:        
            window.mainloop()
        
        
    
    except rospy.ROSInterruptException:
    
        rospy.loginfo("node terminated.")