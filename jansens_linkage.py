import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
import time
from matplotlib.widgets import Slider

def calc_joint(a,b,c,d,e,f,g,h,i,j,k,l,m, theta):
    #Variables of interest
    S0 = math.sqrt(a**2 + l**2)
    beta1 = math.atan(l / a)
    beta2 = np.pi + beta1 - theta
    S1 = math.sqrt(S0**2 + m**2 - 2 * S0 * m * math.cos(beta2))
    beta3 = math.atan((m * math.sin(beta2)) / (S0 - m * math.cos(beta2)))
    beta4 = math.acos((S1**2 + b**2 - j**2)/(2 * S1 * b))
    theta1 = np.pi/2 - beta1 - beta3 - beta4
    beta6 = math.acos((S1**2 + c**2 - k**2)/(2 * S1 * c))
    beta7 = (3*np.pi/2)- beta4 - beta6
    S2 = math.sqrt(d**2 + c**2 - 2 * d * c * math.cos(beta7))
    beta11 = math.acos((c**2 + S2**2 - d**2)/(2 * c * S2)) + math.acos((g**2 + S2**2 - f**2)/(2 * g * S2))
    beta14 = beta6 - beta1 - beta3 - beta11
    #Joint positions
    #fixed point, centre of motor
    A = (0, 0)
    #fixed linkage reference point
    B = (-a, -l)
    #directly driven joint
    H = (m*np.cos(theta), m*np.sin(theta))

    E = (B[0] + b * math.cos(beta4 + beta3 + beta1), B[1] + b * math.sin(beta4 + beta3 + beta1))

    D = (B[0] - d * math.cos(theta1), B[1] + d * math.sin(theta1))

    C = (B[0] + c * math.cos(beta1 + beta3 - beta6), B[1] + c * math.sin(beta1 + beta3 - beta6))

    F = (C[0] - g * math.cos(beta14), C[1] +g * math.sin(beta14))

    G = (C[0] - i * math.cos(np.pi/2 - beta14), C[1] - i * math.sin(np.pi/2 - beta14))

    joint_pos = ([A, B, C, D, E, F, G, H])

    return joint_pos, beta11, beta14


#Linkage lengths
a = 38 #horizontal distance between fixed points
b = 41.3 #t1
c = 39.3
d = 40.1 #t1
e = 57.56474616 #t1
f = 39.4
g = 36.7 #t2
h = 61.22001307 #t2
i = 49 #t2
j = 50
k = 61.9
l = 7.8 #vertical distance between fixed point
m = 15 #length of driving link
real_lengths = ([a, b, c, d, e, f, g, h, i, j, k, l, m])
joint_no = 8
link_no = 11


theta = np.linspace(0,2*np.pi, 360)

joint_pairs = [(0, 7), (1, 4), (1, 3), (1, 2), (2, 5), (2, 6), (2, 7), (3, 4), (3, 5), (4, 7), (5, 6)]
#joint pairs, m, b, d, c, g, i, k, e, f,  j, h
matching_links = ([m, b, d, c, g, i, k, e, f, j, h])
link_letter = ['m', 'b', 'd', 'c', 'g', 'i', 'k', 'e', 'f', 'j', 'h']

data = [[0]*joint_no for _ in range(len(theta))]
#print(data)
#generate data for theta 0 -> 360
# for index, ii in enumerate(theta):
#     data[index], _, _ = calc_joint(a,b,c,d,e,f,g,h,i,j,k,l,m, ii)

#set up figure for animation 
fig1, ax1 = plt.subplots()
plt.subplots_adjust(left = 0.25, bottom = 0.25) #make space for sliders
ax1.grid()
ax1.set_xlim(-90, 20)
ax1.set_ylim(-100, 40)
ax1.set_aspect('equal')
plt.xlabel('X axis (mm)')
plt.ylabel('Y axis (mm)')
plt.title('Jansens Linkage')

#set up empty plots to be filled each frame
scat = ax1.scatter([],[])
lines = [ax1.plot([],[])[0] for _ in range(link_no)]#one plot per line
gait = ax1.plot([],[])

#Slider axes
ax_a = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_l = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')

#make slider
s_a = Slider(ax_a, 'Fixed a', 30, 70, valinit = a)
s_l = Slider(ax_l, 'Fixed l', 5, 50, valinit = l)

ani = None
store = 0
mm = 0

def update_fig(frame):
    global ani
    global mm
    #calculate all positions of joints for all theta
    for index, ii in enumerate(theta):
        #at theta = ii, calculate joint positions [(x,y), (x1,y1),....)]
        data[index], _, _ = calc_joint(s_a.val,b,c,d,e,f,g,h,i,j,k,s_l.val,m, ii)

    

    scat.set_offsets(data[frame])
    error = [None for _ in range(link_no)]

    #update lines
    for index, (ii, jj) in enumerate(joint_pairs):
        line_x = [data[frame][ii][0], data[frame][jj][0]]
        line_y = [data[frame][ii][1], data[frame][jj][1]]

        length = math.sqrt((line_x[1] - line_x[0]) ** 2 + (line_y[1] - line_y[0]) ** 2)
        #angle = (math.atan((line_y[1] - line_y[0])/(line_x[1] - line_x[0])))*360/(2*np.pi)
        
        error[index] = matching_links[index] - length
        if abs(error[index]) > 0.0000001:
            colour = 'red'
            
        else:
            colour = 'blue'
        lines[index].set_data(line_x, line_y)
        lines[index].set_color(colour)
    
    

    return scat, *lines

def pause_animation(duration):
    ani.event_source.stop()
    plt.pause(duration)
    ani.event_source.start()

num_frames = len(theta)
ani = FuncAnimation(fig1, update_fig, frames = num_frames, interval = 2, blit = True)
#print(data[3][4][0])
plt.figure(fig1.number)
plt.show(block = False)

# Global variable to track animation state
anim_running = True

# Function to pause/resume animation
def onClick(event):
    global anim_running
    if anim_running:
        ani.event_source.stop()
        anim_running = False
    else:
        ani.event_source.start()
        anim_running = True

# Connect pause/resume function to mouse click event
fig1.canvas.mpl_connect('button_press_event', onClick)

plt.show()

