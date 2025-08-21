"""
filename: joint_moving_plot
author: Dr. Kaustubh Jalundhwala
Date: 2024/08/09
description: File for plotting a moving jansens linkage and the corresponding stride pattern created. 
The length of each linkage can be adjusted. Hover over the joint while in motion and it will highlight red.
Click and drage the joint to adjust linkage length. If not image appears the linkage length is unsolvable.

"""

import numpy as np
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
import time
from matplotlib.widgets import Slider, Button

#returns the x, y coordinates of each joint based on the input link lengths, in an array
def calc_joint(link_lengths, theta):
    #pull out links from link_length array
    a = link_lengths[0]
    b = link_lengths[1]
    c = link_lengths[2]
    d = link_lengths[3]
    e = link_lengths[4]
    f = link_lengths[5]
    g = link_lengths[6]
    h = link_lengths[7]
    i = link_lengths[8]
    j = link_lengths[9]
    k = link_lengths[10]
    l = link_lengths[11]
    m = link_lengths[12]

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

    #math error on beta11
    try:
        beta11 = math.acos((c**2 + S2**2 - d**2)/(2 * c * S2)) + math.acos((g**2 + S2**2 - f**2)/(2 * g * S2))
    except:
        return None
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

    return joint_pos

#find the length between two points, returns length
def hyp_find(A, B):
    hyp = math.sqrt((A[0] - B[0])**2 + (A[1] - B[1])**2)

    return hyp

# A/B angle
def get_angle(A, B):
    angle = math.atan((B[1]-A[1])/(B[0]-A[0]))
    return angle
#feed it joint positions and spit out the link lengths
#joint positions as x,y coordinates and theta
def calc_links(joint_pos):
    A = joint_pos[0]
    B = joint_pos[1]
    C = joint_pos[2]
    D = joint_pos[3]
    E = joint_pos[4]
    F = joint_pos[5]
    G = joint_pos[6]
    H = joint_pos[7]


    a = - B[0]
    b = hyp_find(B, E)
    c = hyp_find(B, C)
    d = hyp_find(B, D)
    e = hyp_find(D, E)
    f = hyp_find(D, F)
    g = hyp_find(F, C)
    h = hyp_find(F, G)
    i = hyp_find(C, G)
    j = hyp_find(E, H)
    k = hyp_find(C, H)
    l = - B[1]
    m = hyp_find(A, H)

    theta = math.atan((H[1] - A[1]) / (H[0] - A[0]))

    link_lengths = ([a, b, c, d, e, f, g, h, i, j, k, l, m])
    return link_lengths, theta

#returns array of lengths based on research paper
def get_og_length():
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

    length_array = ([a, b, c, d, e, f, g, h, i, j, k, l, m])
    return length_array

#returns array of joint positions based on original lengths and starting theta
def get_og_joint_pos():
    global theta_start

    original_lengths = get_og_length()
    original_joint_pos = calc_joint(original_lengths, theta_start)

    return original_joint_pos

#tests if the link lengths and joints match, return 1 on error
def test_length(lengths, joints):
    global theta_start
    e = 0
    link_letter = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm']
    link_length_test, theta_test  = calc_links(joints)
    #check test
    for index, ii in enumerate(link_length_test):
        error = lengths[index] - ii
        if error> 0.00001:        
            print(f"Error in link: {link_letter[index]} = {error}")
            e = 1

    #check theta
    if theta_start - theta_test > 0.000001:
        print(f"Error in theta = {theta_start-theta_test} ")
        e = 1

    return e

#tests if the links selected can rotate, return None if error
def test_rotation(link_lengths):
    theta = np.linspace(0, 2*np.pi, 360)
    for ii in theta:
        test = calc_joint(link_lengths, ii)
        if test == None:
            print(f"Error in rotation theta = {ii*180/np.pi}")
            return None
    return 1

#checks the selected joint positions gives a valid solution
def test_new_pos(new_joint_pos):

    e = 0
    link_lengths, _ = calc_links(new_joint_pos)
    #print(link_lengths[1])
    validity = test_rotation(link_lengths)
    if validity == None:
        print("Joint location INVALID")
        e = 1
    #print("VALID Geometry")

    return e


#returns valid coordinates for G based on the position of C
def find_valid_G(C, G):
    g_angle = math.atan((C[0] - G[0])/(C[1]-G[1]))
    G_vals = np.linspace(0,50, 50)
    G_coords = np.zeros(2, len(G_vals))
    for index, ii in enumerate(G_vals):
        Gx = C[0] - ii*math.sin(g_angle)
        Gy = C[1] - ii*math.cos(g_angle)
        G_coords[index] = ([Gx, Gy])
    return G_coords


#moves a x,y coordinate by an amount at an angle, returns new x, y coordinate
def change_pos(old, change, angle):
    new_x = old[0] + change*math.cos(angle)
    new_y = old[1] + change*math.sin(angle)

    new = ([new_x, new_y])
    return  new

def change_G(old_joint_pos, change):
    
    g_ang = get_angle(old_joint_pos[2], old_joint_pos[6])
    #print(g_ang*360/(2*np.pi))
    G_new = change_pos(old_joint_pos[6], change, g_ang)
    new_joint_pos = old_joint_pos
    new_joint_pos[6] = G_new
    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos

def change_F(old_joint_pos, change):
    new_joint_pos = old_joint_pos
    f_ang = get_angle(new_joint_pos[2], new_joint_pos[5])
    #print(f"{f_ang*360/(2*np.pi)} degrees")
    F_new = change_pos(new_joint_pos[5], change, f_ang)
    
    new_joint_pos[5] = F_new
    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos

def change_D(old_joint_pos, change):
    new_joint_pos = old_joint_pos
    d_ang = get_angle(new_joint_pos[1], new_joint_pos[3])
    #print(f"{d_ang*360/(2*np.pi)} degrees")
    new_joint_pos[3] = change_pos(new_joint_pos[3], change, d_ang)
    
    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos

def change_E(old_joint_pos, change):
    new_joint_pos = old_joint_pos
    e_ang = get_angle(new_joint_pos[1], new_joint_pos[4])
    #print(f"{e_ang*360/(2*np.pi)} degrees")
    new_joint_pos[4] = change_pos(new_joint_pos[4], change, e_ang)
    
    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos

def change_H(old_joint_pos, change):
    new_joint_pos = old_joint_pos
    h_ang = get_angle(new_joint_pos[0], new_joint_pos[7])
    #print(f"{h_ang*360/(2*np.pi)} degrees")
    new_joint_pos[7] = change_pos(new_joint_pos[7], change, h_ang)
    
    e = test_new_pos(new_joint_pos)
    if e == 1:
        print(e)
        return None
    
    return new_joint_pos

#Change C
def change_C(old_joint_pos, x, y):
    C_new = ([old_joint_pos[2][0] + x, old_joint_pos[2][1] + y])
    F_new = ([old_joint_pos[5][0] + x, old_joint_pos[5][1] + y])
    G_new = ([old_joint_pos[6][0] + x, old_joint_pos[6][1] + y])
    new_joint_pos = old_joint_pos
    new_joint_pos[2] = C_new
    new_joint_pos[5] = F_new
    new_joint_pos[6] = G_new

    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos

def change_B(old_joint_pos, x, y):
    new_joint_pos = old_joint_pos
    B_new = ([new_joint_pos[1][0] + x, new_joint_pos[1][1] + y])
    D_new = ([new_joint_pos[3][0] + x, new_joint_pos[3][1] + y])
    E_new = ([new_joint_pos[4][0] + x, new_joint_pos[4][1] + y])
    
    new_joint_pos[1] = B_new
    new_joint_pos[3] = D_new
    new_joint_pos[4]= E_new

    e = test_new_pos(new_joint_pos)
    if e == 1:
        return None
    
    return new_joint_pos



#fixed
joint_no = 8
link_no = 11
link_letter = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm']
theta_start = np.pi/4


original_lengths = get_og_length()
original_joint_pos = get_og_joint_pos()
current_lengths = original_lengths

# debugging code, not needed for now
if False:
    #print(f"First C = {original_joint_pos[2]}, First G = {original_joint_pos[6]}")
    #Change point g, -ve numbers make it go further from origin
    new_joint_pos = change_F(original_joint_pos[:], -5)
    new_joint_pos_1 = change_G(new_joint_pos[:], -5)
    new_joint_pos_2 = change_C(new_joint_pos_1[:], 5, 5)

    new_joint_pos_3 = change_D(original_joint_pos[:], -5)
    new_joint_pos_4 = change_E(new_joint_pos_3[:], -5)
    new_joint_pos_5 = change_B(new_joint_pos_4[:], -5, -15)

    new_joint_pos_6 = change_H(original_joint_pos[:], 0.3)

    if new_joint_pos_6[0][0] == None:
        print("Maths gone wrong")

    print(f" \nOriginal: C = ({original_joint_pos[2][0]:.4f}, {original_joint_pos[2][1]:.4f}), F = ({original_joint_pos[5][0]:.4f} {original_joint_pos[5][1]:.4f}), G = ({original_joint_pos[6][0]:.4f} {original_joint_pos[6][1]:.4f})")
    print(f" Change F: C = ({new_joint_pos[2][0]:.4f}, {new_joint_pos[2][1]:.4f}), F = ({new_joint_pos[5][0]:.4f} {new_joint_pos[5][1]:.4f}), G = ({new_joint_pos[6][0]:.4f} {new_joint_pos[6][1]:.4f})")
    print(f" Cha F, G: C = ({new_joint_pos_1[2][0]:.4f}, {new_joint_pos_1[2][1]:.4f}), F = ({new_joint_pos_1[5][0]:.4f} {new_joint_pos_1[5][1]:.4f}), G = ({new_joint_pos_1[6][0]:.4f} {new_joint_pos_1[6][1]:.4f})")
    print(f" F, G, C:  C = ({new_joint_pos_2[2][0]:.4f}, {new_joint_pos_2[2][1]:.4f}), F = ({new_joint_pos_2[5][0]:.4f} {new_joint_pos_2[5][1]:.4f}), G = ({new_joint_pos_2[6][0]:.4f} {new_joint_pos_2[6][1]:.4f})")

    print(f" \nOriginal: B = ({original_joint_pos[1][0]:.4f}, {original_joint_pos[1][1]:.4f}), D = ({original_joint_pos[3][0]:.4f} {original_joint_pos[3][1]:.4f}), E = ({original_joint_pos[4][0]:.4f} {original_joint_pos[4][1]:.4f})")
    print(f"Change D: B = ({new_joint_pos_3[1][0]:.4f}, {new_joint_pos_3[1][1]:.4f}), D = ({new_joint_pos_3[3][0]:.4f} {new_joint_pos_3[3][1]:.4f}), E = ({new_joint_pos_3[4][0]:.4f} {new_joint_pos_3[4][1]:.4f})")
    print(f"Cha D, E: B = ({new_joint_pos_4[1][0]:.4f}, {new_joint_pos_4[1][1]:.4f}), D = ({new_joint_pos_4[3][0]:.4f} {new_joint_pos_4[3][1]:.4f}), E = ({new_joint_pos_4[4][0]:.4f} {new_joint_pos_4[4][1]:.4f})")
    print(f"D, E, B: B = ({new_joint_pos_5[1][0]:.4f}, {new_joint_pos_5[1][1]:.4f}), D = ({new_joint_pos_5[3][0]:.4f} {new_joint_pos_5[3][1]:.4f}), E = ({new_joint_pos_5[4][0]:.4f} {new_joint_pos_5[4][1]:.4f})")

    print(f" \nOriginal: A = ({original_joint_pos[0][0]:.4f}, {original_joint_pos[0][1]:.4f}), H = ({original_joint_pos[7][0]:.4f} {original_joint_pos[7][1]:.4f})")
    print(f" \nChange H: A = ({new_joint_pos_6[0][0]:.4f}, {new_joint_pos_6[0][1]:.4f}), H = ({new_joint_pos_6[7][0]:.4f} {new_joint_pos_6[7][1]:.4f})")

joint_pairs = [(0, 7), (1, 4), (1, 3), (1, 2), (2, 5), (2, 6), (2, 7), (3, 4), (3, 5), (4, 7), (5, 6)]
joint_update_funcs = [ None, change_B, change_C, change_D, change_E, change_F, change_G, change_H ]
joint_free_direction = [ None, None, None, (1, 3), (1, 4), (2, 5), (2, 6), (0, 7) ]

#joint pairs, m, b, d, c, g, i, k, e, f,  j, h
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
gait = ax1.plot([],[], color='red')[0]

#Slider axes
ax_a = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor='lightgoldenrodyellow')
ax_l = plt.axes([0.25, 0.1, 0.65, 0.03], facecolor='lightgoldenrodyellow')

#make slider
b_a = Button(ax_a, "Inc")
#s_a = Slider(ax_a, 'Fixed a', 30, 70, valinit = original_lengths[0])
s_l = Slider(ax_l, 'Fixed l', 5, 50, valinit = original_lengths[11])

def on_click_inc(evt):
    global current_lengths

    print(current_lengths)
    new_pos = change_G(data[0], 5)
    current_lengths, _ = calc_links(new_pos)
    print(current_lengths)

b_a.on_clicked(on_click_inc)

ani = None
store = 0

def update_data(lengths):
    global data
    #calculate all positions of joints for all theta
    for index, ii in enumerate(theta):
        #at theta = ii, calculate joint positions [(x,y), (x1,y1),....)]
        data[index] = calc_joint(lengths, ii)

def update_fig(fake_frame):
    global anim_running, data, current_frame, current_theta, highlighted_joint, joint_no

    if anim_running:
        current_frame += 1
        if current_frame >= num_frames:
            current_frame = 0

    frame = current_frame

    # update joints
    joint_colours = np.empty(joint_no, dtype=object)
    joint_colours.fill('blue')
    if highlighted_joint != -1:
        joint_colours[highlighted_joint] = 'red'

    scat.set_offsets(data[frame])
    scat.set_color(joint_colours)

    # update links
    for index, (ii, jj) in enumerate(joint_pairs):
        line_x = [data[frame][ii][0], data[frame][jj][0]]
        line_y = [data[frame][ii][1], data[frame][jj][1]]

        length = math.sqrt((line_x[1] - line_x[0]) ** 2 + (line_y[1] - line_y[0]) ** 2)
        #angle = (math.atan((line_y[1] - line_y[0])/(line_x[1] - line_x[0])))*360/(2*np.pi)
        
        lines[index].set_data(line_x, line_y)
        lines[index].set_color('blue')
    
    # update overall path of foot
    gait_x = np.zeros(np.size(theta))
    gait_y = np.zeros(np.size(theta))
    for index, _ in enumerate(theta):
        gait_x[index] = data[index][joint_of_interest][0]
        gait_y[index] = data[index][joint_of_interest][1]
    gait.set_data(
        gait_x,
        gait_y
    )

    return scat, *lines, gait

def pause_animation(duration):
    ani.event_source.stop()
    plt.pause(duration)
    ani.event_source.start()

theta = np.linspace(0,2*np.pi, 360)
data = [[0]*joint_no for _ in range(len(theta))]
update_data(original_lengths)

# the index of the joint to plot the full path
joint_of_interest = 6

# Global variable to track animation state
anim_running = True
current_frame = 0
current_theta = 0
highlighted_joint = -1
next_joint_positions = None
mouse_is_down = False

num_frames = len(theta)
ani = FuncAnimation(fig1, update_fig, frames = num_frames, interval = 2, blit = True)
#print(data[3][4][0])
plt.figure(fig1.number)
plt.show(block = False)

# Function to pause/resume animation
def onMouseDown(event):
    global anim_running, mouse_is_down

    mouse_is_down = True
    anim_running = False

def onMouseUp(event):
    global anim_running, mouse_is_down, next_joint_positions, current_lengths

    #if next_joint_positions != None:
    #    current_lengths, _ = calc_links(next_joint_positions)

    mouse_is_down = False
    anim_running = True

def onMouseMove(evt):
    global highlighted_joint, data, current_frame, next_joint_positions

    if evt.xdata != None:
        if mouse_is_down:
            free_joint = joint_free_direction[highlighted_joint]
            if free_joint != None:
                g_vec = np.array(data[current_frame][free_joint[0]]) - np.array(data[current_frame][free_joint[1]])
                g_vec = g_vec / np.sqrt(np.sum(g_vec**2))

                mouse_vec = np.array((evt.xdata, evt.ydata)) - np.array(data[current_frame][highlighted_joint])
                dx = np.dot(mouse_vec, g_vec)

                change_func = joint_update_funcs[highlighted_joint]
                next_joint_positions = change_func(data[current_frame], dx)
                if next_joint_positions != None:
                    new_lengths, _ = calc_links(next_joint_positions)
                    update_data(new_lengths)
        else:
            min_dist = math.inf
            closest_joint_ix = -1
            for ix in range(0, joint_no):
                dist = hyp_find((evt.xdata, evt.ydata), data[current_frame][ix])
                if dist < min_dist:
                    min_dist = dist
                    closest_joint_ix = ix
            highlighted_joint = closest_joint_ix
    else:
        highlighted_joint = -1

# Connect pause/resume function to mouse click event
fig1.canvas.mpl_connect('button_press_event', onMouseDown)
fig1.canvas.mpl_connect('button_release_event', onMouseUp)
fig1.canvas.mpl_connect('motion_notify_event', onMouseMove)

plt.show()