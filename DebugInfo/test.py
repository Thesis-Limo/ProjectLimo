import yaml
import matplotlib.pyplot as plt
import rospy
from limo_yolo.msg import map

from matplotlib.animation import FuncAnimation

fig, ax = plt.subplots(figsize=(20, 20))
mapp = map
test = False
# update function to update data and plot
def update(frame):
    global mapp, test
    if(test == False):
        return
    # updating the data by adding one more point
    x_values = [-obstacle.point.y for obstacle in mapp.obstacles]
    y_values = [obstacle.point.x for obstacle in mapp.obstacles]
    x_values_goal = [-goal.point.y for goal in mapp.goal]
    y_values_goal = [goal.point.x for goal in mapp.goal]

    ax.clear()  # clearing the axes
    ax.scatter(x_values, y_values, color='r', marker='o', s=10, facecolors='none', label='Obstacles')
    ax.scatter([0], [0], color='b', marker='x', s=10, label='Origin')
    ax.scatter(x_values_goal, y_values_goal, color='g', marker='o', s=10, facecolors='none', label='Goals')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Obstacles')
    ax.grid(True)
    ax.legend()
    fig.canvas.draw()  # forcing the artist to redraw itself
 
def printing(map):
    global mapp, test
    test = True
    mapp = map


if __name__ == "__main__":

    rospy.init_node("testing")
    s = rospy.Subscriber("/map", map, lambda map: printing(map))
    mapp = map
    anim = FuncAnimation(fig, update)
    plt.show()
    rospy.spin()

