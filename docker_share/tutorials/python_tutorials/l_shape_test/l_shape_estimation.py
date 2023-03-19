import numpy as np
import argparse
from typing import List
import matplotlib.pyplot as plt
from math import dist
import math
from numpy.linalg import norm

euclid_thr = 0.3
theta_thr = 12

x_low = -7
x_high = -4
y_low = 5
y_high = 9

x_low_2 = 8.5
x_high_2 = 13
y_low_2 = 3
y_high_2 = 7

fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set(xlim=(x_low_2, x_high_2), xticks=np.arange(x_low_2, x_high_2),
       ylim=(y_low_2, y_high_2), yticks=np.arange(y_low_2, y_high_2))

def read_file(file_name) -> List[List]:
    points = list()
    with open(file_name) as text:
        for line in text:
            data = line.rstrip("\n").split("\t")
            point = list()
            for inst in data:
                chunks = inst.split()
                point = [float(x) for x in chunks]
            points.append(point)
    return points

def get_angle(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)

    ba = a - b
    bc = c - b

    #cosine_angle = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    #angle = np.arccos(cosine_angle)
    ang = math.degrees(math.atan2(c[1]-b[1], c[0]-b[0]) - math.atan2(a[1]-b[1], a[0]-b[0]))
    if abs(ang) > 180:
        ang = 360 - abs(ang)
    else:
        ang = abs(ang)

    #return np.degrees(angle)
    return ang

def geometric_center(candidate):
    data_candidate = np.array(candidate)
    return data_candidate.mean(axis=0)

def find_vertexes(candidate):
    dist_min = float('inf')
    candidate_indices = []
    for i in range(len(candidate) - 1):
        for j in range(len(candidate) - 1 - i):
            p1 = np.asarray(candidate[i])
            p2 = np.asarray(candidate[i+j+1])
            euclid_dist = dist(p1, p2)
            if euclid_dist < euclid_thr and euclid_dist < dist_min:
                dist_min = euclid_dist
                candidate_indices = []
                candidate_indices.append(i)
                candidate_indices.append(i+j+1)
    v_a = geometric_center([candidate[candidate_indices[0]], candidate[candidate_indices[1]]])
    candidate.pop(candidate_indices[0])
    candidate.pop(candidate_indices[1] - 1)
    v_b = []
    if get_angle(v_a, candidate[0], candidate[1]) > get_angle(v_a, candidate[1], candidate[0]):
        v_b = candidate[1]
    else:
        v_b = candidate[0]
    return v_a, np.array(v_b)

def get_eqn_line(p1, p2):
    line = []
    line.append(p1[1] - p2[1])
    line.append(p2[0] - p1[0])
    line.append((p1[0] - p2[0])*p1[1] + (p2[1] - p1[1])*p1[0])
    return line

def get_points_from_eqn(line):
    points = []
    points.append(line[3])
    point = []
    point.append(line[3][0] + 20)
    point.append((line[2] - line[0]*point[0])/(line[1]))
    np.asarray(point)
    points.append(point)
    return points

def get_intersect_from_lines(line_1, line_2):
    point = [(-line_1[1]*line_2[2] + line_2[1]*line_1[2])/(line_1[0]*line_2[1] - line_2[0]*line_1[1]), 
             (-line_1[2]*line_2[0] + line_2[2]*line_1[0])/(line_1[0]*line_2[1] - line_2[0]*line_1[1])]
    print(point)
    return point

def get_intersect_from_lines_2(line_1, line_2):
    point = [(line_1[0]*line_2[2] - line_2[0]*line_1[2])/(line_1[1]*line_2[0] - line_2[1]*line_1[0]), 
             (line_1[2]*line_2[1] - line_2[2]*line_1[1])/(line_1[1]*line_2[0] - line_2[1]*line_1[0])]
    print(point)
    return point

def dist_line(p3, p1, p2):
    #return norm(np.cross(p2-p1, p1-p3))/norm(p2-p1)
    return abs((p2[0]-p1[0])*(p1[1]-p3[1]) - (p1[0]-p3[0])*(p2[1]-p1[1])) / np.sqrt(np.square(p2[0]-p1[0]) + np.square(p2[1]-p1[1]))

def search_vertexes(points):
    abscissa = sorted(points, key=lambda point: point[0])
    ordinate = sorted(points, key=lambda point: point[1])
    vertex_candidate = []
    vertex_candidate.append(abscissa.pop(0))
    vertex_candidate.append(abscissa.pop())
    vertex_candidate.append(ordinate.pop(0))
    vertex_candidate.append(ordinate.pop())
    return find_vertexes(vertex_candidate)

def localize_optimal_corner_point(data_points, v_a, v_b):
    err_min = float('inf')
    N_1 = 0 
    N_2 = 0
    P_best = []
    for i in range(len(data_points)):
        if np.pi / 2 + np.radians(theta_thr) >= abs(np.radians(get_angle(v_a, data_points[i], v_b))) >= np.pi / 2 - np.radians(theta_thr):
            N_t1 = 0 
            N_t2 = 0
            err = 0
            for j in range(len(data_points)):
                if j != i:
                    dist_1 = dist_line(data_points[j], v_a, data_points[i])
                    dist_2 = dist_line(data_points[j], data_points[i], v_b)
                    if dist_1 < dist_2:
                        N_t1 += 1
                        err += dist_1
                    else:
                        N_t2 += 1
                        err += dist_2
            if err <= err_min:
                err_min = err
                P_best = data_points[i]
                N_1 = N_t1
                N_2 = N_t2
    return P_best, N_1, N_2

def localize_optimal_corner_point_old(data_points, v_a, v_b):
    err_min = float('inf')
    N_1 = 0 
    N_2 = 0
    P_best = []
    for i in range(len(data_points)):
        #print(data_points[i], abs(get_angle(v_a, datalinewidth=4_points[i], v_b)))
        if np.pi / 2 + np.radians(theta_thr) >= abs(np.radians(get_angle(v_a, data_points[i], v_b))) >= np.pi / 2 - np.radians(theta_thr):
            N_t1 = 0 
            N_t2 = 0
            err = 0
            for j in range(len(data_points)):
                if j != i:
                    dist_1 = dist_line(data_points[j], v_a, data_points[i])
                    dist_2 = dist_line(data_points[j], data_points[i], v_b)
                    if dist_1 < dist_2:
                        N_t1 += 1
                        err += dist_1
                    else:
                        N_t2 += 1
                        err += dist_2
                    if err <= err_min:
                        #print("New error:", err, data_points[i])
                        err_min = err
                        P_best = data_points[j]
                        N_1 = N_t1
                        N_2 = N_t2
    return P_best, N_1, N_2

def shape_fitting(v_a, v_b, P_best, N_1, N_2, data_points):
    # TODO: Currently not working
    if N_1 > N_2:
        theta_1 = np.arctan2(v_a[1] - P_best[1], v_a[0] - P_best[0])
        theta_2 = theta_1 + np.pi / 2
        ref_1 = v_b
        ref_2 = v_a
    else:
        theta_2 = np.arctan2(P_best[1] - v_b[1], P_best[0] - v_b[0])
        theta_1 = theta_2 + np.pi / 2
        ref_2 = v_a
        ref_1 = v_b
    #print(N_1, N_2)
    #print(theta_1, theta_2)
    #print(np.cos(theta_1), np.sin(theta_1))
    #print(data_points)
    p1 = np.array([np.cos(theta_1), np.sin(theta_1)])
    p2 = np.array([np.cos(theta_2), np.sin(theta_2)])
    #print(p1)
    #print(p2)
    c1 = np.dot(data_points, p1)
    c2 = np.dot(data_points, p2)
    print(c1)
    print(c2)
    #print(np.cos(theta_1), np.sin(theta_1), min(c1))
    lines = []
    lines.append([np.cos(theta_1), np.sin(theta_1), min(c1), ref_2])
    lines.append([np.cos(theta_2), np.sin(theta_2), min(c2), ref_1])
    lines.append([np.cos(theta_1), np.sin(theta_1), max(c1), ref_1])
    lines.append([np.cos(theta_2), np.sin(theta_2), max(c2), ref_2])
    return lines

def shape_fitting2(v_a, v_b, P_best, N_1, N_2, data_points):
    # TODO: Currently not working
    if N_1 > N_2:
        theta_1 = np.arctan2(v_a[1] - P_best[1], v_a[0] - P_best[0])
        theta_2 = theta_1 + np.pi / 2
        ref_1 = v_b
        ref_2 = v_a
    else:
        theta_2 = np.arctan2(P_best[1] - v_b[1], P_best[0] - v_b[0])
        theta_1 = theta_2 + np.pi / 2
        ref_2 = v_a
        ref_1 = v_b
    #print(N_1, N_2)
    #print(theta_1, theta_2)
    #print(np.cos(theta_1), np.sin(theta_1))
    #print(data_points)
    p1 = np.array([np.cos(theta_1), np.sin(theta_1)])
    p2 = np.array([np.cos(theta_2), np.sin(theta_2)])
    #print(p1)
    #print(p2)
    c1 = np.dot(data_points, np.array([p1[1], p1[0]]))
    c2 = np.dot(data_points, np.array([p2[1], p2[0]]))
    print(c1)
    print(c2)
    #print(np.cos(theta_1), np.sin(theta_1), min(c1))
    lines = []
    lines.append([np.cos(theta_1), np.sin(theta_1), min(c1), ref_2])
    lines.append([np.cos(theta_2), np.sin(theta_2), min(c2), ref_1])
    lines.append([np.cos(theta_1), np.sin(theta_1), max(c1), ref_1])
    lines.append([np.cos(theta_2), np.sin(theta_2), max(c2), ref_2])
    return lines

def plot_points(points, symbol):
    for point in points:
        ax.plot(point[0], point[1], symbol)

def draw_line(a, b, symbol):
    ax.plot([a[0], b[0]], [a[1], b[1]], symbol)

def draw_line_2(p1, p2, symbol):
    ax.axline((p1[0], p1[1]), (p2[0], p2[1]), linewidth=2, color="black")

def draw_line_3(p1, p2, symbol):
    ax.axline((p1[1], p1[0]), (p2[1], p2[0]), linewidth=2, color="black")

def draw_lines(box_shape, symbol):
    x_range = [-7, -2]
    y_range = []
    
    for line in box_shape:
        for i in x_range:
            y_range.append((line[2] - line[0] * i)/line[1])
        print(y_range, line)
        ax.plot(x_range, y_range, symbol)
        y_range = []

def draw_box(box_shape, symbol):
    for line in box_shape:
        ref_points = get_points_from_eqn(line)
        ax.plot([ref_points[0][0], ref_points[1][0]], [ref_points[0][1], ref_points[1][1]], symbol)

def draw_box_2(box_shape, symbol):
    for line in box_shape:
        ax.axline((line[3][0], line[3][1]), slope=(-line[0]/line[1]), color="black")

def draw_box_3(box_shape, symbol):
    p1 = get_intersect_from_lines(box_shape[0], box_shape[3])
    p2 = get_intersect_from_lines(box_shape[0], box_shape[1])
    p3 = get_intersect_from_lines(box_shape[1], box_shape[2])
    p4 = get_intersect_from_lines(box_shape[2], box_shape[3])
    draw_line_2(p1, p2, symbol)
    draw_line_2(p2, p3, symbol)
    draw_line_2(p3, p4, symbol)
    draw_line_2(p4, p1, symbol)

def draw_box_4(box_shape, symbol):
    p1 = get_intersect_from_lines(box_shape[0], box_shape[3])
    p2 = get_intersect_from_lines(box_shape[0], box_shape[1])
    p3 = get_intersect_from_lines(box_shape[1], box_shape[2])
    p4 = get_intersect_from_lines(box_shape[2], box_shape[3])
    draw_line(p1, p2, symbol)
    draw_line(p2, p3, symbol)
    draw_line(p3, p4, symbol)
    draw_line(p4, p1, symbol)

def show_plot():
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file')
    args = parser.parse_args()

    data_points = read_file(args.file)
    data_points = np.asarray(data_points)
    plot_points(data_points, 'ob')
    v_a, v_b = search_vertexes(data_points)
    plot_points([v_a, v_b], '*r' )
    P_best, N_1, N_2 = localize_optimal_corner_point(data_points, v_a, v_b)
    plot_points([P_best], 'og')
    box_shape = shape_fitting(v_a, v_b, P_best, N_1, N_2, data_points)
    box_shape2 = shape_fitting2(v_a, v_b, P_best, N_1, N_2, data_points)
    #draw_line(v_a, P_best, 'g')
    #draw_lines(box_shape, 'g')
    
    #draw_box_2(box_shape, 'g')
    draw_box_4(box_shape, 'black')

    #P_best_old, _, _ = localize_optimal_corner_point_old(data_points, v_a, v_b)
    #print(P_best_old)
    #plot_points([P_best_old], 'xy')
    show_plot()

if __name__ == '__main__':
    main()