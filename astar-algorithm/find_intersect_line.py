# A Python3 program to find if 2 given line segments intersect or not
import matplotlib.pyplot as plt


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    # Given three colinear points p, q, r, the function checks if


# point q lies on line segment 'pr'
def onSegment(p, q, r):
    """
    The function `onSegment` checks if point `q` lies on the line segment defined by points `p` and `r`.

    :param p: The `onSegment` function you provided checks if point `q` lies on the line segment defined
    by points `p` and `r`. The function returns `True` if `q` is on the segment and `False` otherwise
    :param q: It seems like you were about to provide the parameters for the function `onSegment`, but
    the information is missing. Could you please provide the values for the parameters `p`, `q`, and `r`
    so I can assist you further?
    :param r: The `onSegment` function you provided checks if point `q` lies on the line segment defined
    by points `p` and `r`. The function returns `True` if `q` lies on the segment and `False` otherwise
    :return: The function `onSegment` is returning a boolean value - `True` if point `q` lies on the
    line segment defined by points `p` and `r`, and `False` otherwise.
    """
    if (
        (q.x <= max(p.x, r.x))
        and (q.x >= min(p.x, r.x))
        and (q.y <= max(p.y, r.y))
        and (q.y >= min(p.y, r.y))
    ):
        return True
    return False


def orientation(p, q, r):
    """
    The function determines the orientation of three points in a plane as either colinear, clockwise, or
    counterclockwise.

    :param p: The `p`, `q`, and `r` parameters in the `orientation` function represent three points in a
    2D plane. The function calculates the orientation of these points relative to each other
    :param q: The parameters `p`, `q`, and `r` represent three points in a 2D plane. Each point has an
    `x` and `y` coordinate. The function `orientation` calculates the orientation of these three points
    relative to each other
    :param r: The `orientation` function you provided calculates the orientation of an ordered triplet
    of points `(p, q, r)` in a 2D plane. The function returns 0 if the points are collinear, 1 if they
    are in a clockwise orientation, and 2 if they are in a
    :return: The function `orientation(p, q, r)` returns the orientation of the ordered triplet (p, q,
    r) as one of the following values:
    - 0: Colinear points
    - 1: Clockwise points
    - 2: Counterclockwise points
    """
    # to find the orientation of an ordered triplet (p,q,r)
    # function returns the following values:
    # 0 : Colinear points
    # 1 : Clockwise points
    # 2 : Counterclockwise

    # See https://www.geeksforgeeks.org/orientation-3-ordered-points/amp/
    # for details of below formula.

    val = (float(q.y - p.y) * (r.x - q.x)) - (float(q.x - p.x) * (r.y - q.y))
    if val > 0:

        # Clockwise orientation
        return 1
    elif val < 0:

        # Counterclockwise orientation
        return 2
    else:

        # Colinear orientation
        return 0


# The main function that returns true if
# the line segment 'p1q1' and 'p2q2' intersect.
def doIntersect(p1, q1, p2, q2):
    """
    The function `doIntersect` determines if two line segments intersect based on their orientations and
    positions.

    :param p1: In the provided code snippet, the function `doIntersect` is checking if two line segments
    defined by points `p1`, `q1` and `p2`, `q2` intersect with each other
    :param q1: It seems like you were about to provide more information about the parameters, but it got
    cut off. Could you please provide the complete information about the parameters q1, p1, q2, and p2
    so that I can assist you further with the `doIntersect` function?
    :param p2: It seems like you have provided the function `doIntersect` which checks if two line
    segments intersect. However, you have not provided the definition for the `orientation` and
    `onSegment` functions that are being used within `doIntersect`
    :param q2: It seems like you have provided the function `doIntersect` which checks if two line
    segments intersect. However, you have not provided the definition or values for the parameter `q2`.
    Could you please provide the definition or values for `q2` so that I can assist you further with
    using
    :return: The function `doIntersect` is checking if two line segments defined by points `p1, q1` and
    `p2, q2` intersect or not. The function returns `True` if the line segments intersect, and `False`
    if they do not intersect.
    """
    # Find the 4 orientations required for
    # the general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if (o1 != o2) and (o3 != o4):
        return True

    # Special Cases

    # p1 , q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0) and onSegment(p1, p2, q1):
        return True

    # p1 , q1 and q2 are colinear and q2 lies on segment p1q1
    if (o2 == 0) and onSegment(p1, q2, q1):
        return True

    # p2 , q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0) and onSegment(p2, p1, q2):
        return True

    # p2 , q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0) and onSegment(p2, q1, q2):
        return True

    # If none of the cases
    return False


def plot_line(p1, q1, p2, q2):
    plt.plot([p1.x, q1.x], [p1.y, q1.y], "-b")
    plt.plot([p2.x, q2.x], [p2.y, q2.y], "-r")

    plt.axis([0, 100, 0, 100])
    # This code is contributed by Ansh Riyal
    plt.grid(True)
    plt.pause(0.0001)  # Need for Mac
    plt.show()


if __name__ == "__main__":
    try:
        # Driver program to test above functions:
        p1 = Point(90, 30)
        q1 = Point(75, 60)
        p2 = Point(80, 20)
        q2 = Point(80, 50)

        if doIntersect(p1, q1, p2, q2):
            print("Yes")
        else:
            print("No")
        plot_line(p1, q1, p2, q2)

        p1 = Point(10, 0)
        q1 = Point(0, 10)
        p2 = Point(0, 0)
        q2 = Point(10, 10)

        if doIntersect(p1, q1, p2, q2):
            print("Yes")
        else:
            print("No")
        plot_line(p1, q1, p2, q2)
        p1 = Point(1, 1)
        q1 = Point(8, 1)
        p2 = Point(6, 0)
        q2 = Point(8, 1.2)

        if doIntersect(p1, q1, p2, q2):
            print("Yes")
        else:
            print("No")
        plot_line(p1, q1, p2, q2)
        plt.show()
    except:
        print("Some_thing_went_wrong")
