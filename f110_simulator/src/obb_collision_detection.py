import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import math

pi = math.pi


# separating axis theorem
def is_separating_axis(o, p1, p2):
    """
    Return True and the push vector if o is a separating axis of p1 and p2.
    Otherwise, return False and None.
    """
    min1, max1 = float('+inf'), float('-inf')
    min2, max2 = float('+inf'), float('-inf')

    for v in p1:
        projection = np.dot(v, o)
        min1 = min(min1, projection)
        max1 = max(max1, projection)

    for v in p2:
        projection = np.dot(v, o)
        min2 = min(min2, projection)
        max2 = max(max2, projection)

    if max1 >= min2 and max2 >= min1:
        return False, None
    else:
        d = min(abs(min2 - max1), abs(min1 - max2))
        return True, d


def plot_rectangle(ob):
    """create the rectangle for plot

    Parameters
    ----------
    ob : 1*5 ndarray
        ob = [x,y,yaw,length,width]

    Returns
    -------
    rect
        rectangle for plot
    """
    theta = ob[2]
    rotation_matrix = np.array(
        [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    xy = np.array([[ob[4] / 2], [ob[3] / 2]])
    xy = np.matmul(rotation_matrix, xy)
    xy = np.array([[ob[0]], [ob[1]]]) - xy
    rect = Rectangle((xy[0, 0], xy[1, 0]), ob[4], ob[3], angle=ob[2]
                     * 180 / pi, linewidth=2, edgecolor='k', facecolor='c')
    return rect


class Obstacle:

    def __init__(self, ob):
        """
        Parameters
        ----------
        ob: obstacle description [x,y,yaw,length,width]
        """
        self.ob = ob
        self.vertices = self._cal_vertices()
        self.edges = self._get_edges()
        self.orthogonal = self._get_orthogonal()

    def _cal_vertices(self):
        """
        calculate the two rectangle vertices position.
        """
        vertices = np.zeros((4, 2))
        theta = self.ob[2]
        rotation_matrix = np.array(
            [[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        xy1 = np.array([[self.ob[4] / 2], [-self.ob[3] / 2]])
        xy1 = np.matmul(rotation_matrix, xy1).T[0]
        xy2 = np.array([[self.ob[4] / 2], [self.ob[3] / 2]])
        xy2 = np.matmul(rotation_matrix, xy2).T[0]

        vertices[0, :] = np.array([self.ob[0] + xy1[0], self.ob[1] + xy1[1]])
        vertices[1, :] = np.array([self.ob[0] + xy2[0], self.ob[1] + xy2[1]])
        vertices[2, :] = np.array([self.ob[0] - xy1[0], self.ob[1] - xy1[1]])
        vertices[3, :] = np.array([self.ob[0] - xy2[0], self.ob[1] - xy2[1]])

        return vertices

    def _get_edges(self):
        """
        Return the vectors for the edges of the rectangle.
        """
        edges = [
            self.vertices[1] - self.vertices[0],
            self.vertices[2] - self.vertices[1]
        ]
        return edges

    def _get_orthogonal(self):
        """
        Return a 90-degree counter-clockwise rotation of the vector v.
        """
        return [np.array([-v[1], v[0]]) / np.sqrt(v[0]**2 + v[1]**2) for v in self.edges]


def collision_check(ob1, ob2):
    """
    Return True and the MPV if the shapes collide. Otherwise, return False and
    None.

    Parameters
    ----------
    ob1: Obstacle
        the instance of Obstacle
    ob2: Obstacle
        the instance of Obstacle

    Returns
    -------
    is_collision: bool
        True if there is a collision
    d_max: float or None
        The maximum distance between ob1 and ob2 if there is a collision. Otherwise, it is None
    """
    orthogonal = ob1.orthogonal + ob2.orthogonal
    d_max = -1
    for o in orthogonal:

        separates, d = is_separating_axis(o, ob1.vertices, ob2.vertices)

        if separates:
            # they do not collide and there is no push vector
            if d > d_max:
                d_max = d

    if d_max > 0:
        return False, d_max

    return True, None


if __name__ == "__main__":
    # define two rectangle ob = [x,y,yaw,width,length]

    ob1 = [1, 0.5, 0, 1.0, 2.0]
    ob2 = [1, 7.5, np.pi/2, 1.0, 2.0]
    ob = np.array([ob1, ob2])
    Obs1 = Obstacle(ob1)
    Obs2 = Obstacle(ob2)

    # calculate the vertices
    detection, dist = collision_check(Obs1, Obs2)
    print(f"The detection is {detection}")
    print(f"The distance between two boxes is {dist}")

    # visualize the two bounding box
    ax = plt.gca()
    plt.plot(ob[:, 0], ob[:, 1], "xk")
    for i in range(ob.shape[0]):
        rect = plot_rectangle(ob[i, :])
        ax.add_patch(rect)

    # plot the edge of the rectangle
    for i in range(4):
        plt.plot(Obs1.vertices[i, 0], Obs1.vertices[i, 1], "xr")
        plt.plot(Obs2.vertices[i, 0], Obs2.vertices[i, 1], "xr")

    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.axis("equal")
    plt.grid(False)
    plt.show()
