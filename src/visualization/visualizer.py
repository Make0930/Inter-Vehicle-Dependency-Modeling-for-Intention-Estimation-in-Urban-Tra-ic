import matplotlib.pyplot as plt
from math import sqrt, pow, atan2, pi, floor, cos


def quatmsg2FastYaw(quatmsg):
    # type: (Quaternion) -> float
    """ quatmsg2euler seems to be really slow, so this only computes yaw
        Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    siny_cosp = +2.0 * (quatmsg.w * quatmsg.z + quatmsg.x * quatmsg.y)
    cosy_cosp = +1.0 - 2.0 * (quatmsg.y * quatmsg.y + quatmsg.z * quatmsg.z)
    return atan2(siny_cosp, cosy_cosp)

def draw_ll_map(ll_map, paths_with_info, object_states, critical_areas):
    # type: (LaneletMap, PathsWithInfo, typing.Dict[ObjectId, ObjectState], CriticalAreas) -> None

    # draw debug plot - not very stable due to threading problems

    # draw objects
    for obj_id in paths_with_info:

        # draw object
        object_state = object_states[obj_id]  # type: ObjectState
        obj_yaw = quatmsg2FastYaw(object_state.motion_state.pose.pose.orientation)
        plt.plot(object_state.motion_state.pose.pose.position.x,
                  object_state.motion_state.pose.pose.position.y,
                  marker=(3, 0, obj_yaw/(2*3.14159)*360-90.), markersize=20, linestyle='None')

        # alternative: tilted diamond (sometimes causes crash of plt)
        # marker = mpl.markers.MarkerStyle(marker="d")
        # marker._transform = marker.get_transform().rotate_deg(obj_yaw / (2 * 3.14159) * 360 - 90.)
        # plt.scatter(object_state.motion_state.pose.pose.position.x,
        #             object_state.motion_state.pose.pose.position.y,
        #             marker=marker, s=100)

    # draw lanelets
    added_label = False
    for ll in ll_map.laneletLayer:
        if not added_label:
            plt.plot([p.x for p in ll.leftBound], [p.y for p in ll.leftBound], 'r--', label="left boundary")
            plt.plot([p.x for p in ll.rightBound], [p.y for p in ll.rightBound], 'g-.', label="right boundary")
            added_label = True
        else:
            plt.plot([p.x for p in ll.leftBound], [p.y for p in ll.leftBound], 'r--')
            plt.plot([p.x for p in ll.rightBound], [p.y for p in ll.rightBound], 'g-.')

    # draw critical areas
    for ca in critical_areas:
        circle = plt.Circle((ca.x, ca.y), ca.radius, facecolor='r', edgecolor="black")
        plt.gca().add_artist(circle)
        plt.gca().text(ca.x, ca.y, ca.description, style='italic',
                       bbox={'facecolor': 'red', 'alpha': 0.5, 'pad': 3})

    plt.axis('equal')
    plt.ylabel("$y$ [m]")
    plt.xlabel("$x$ [m]")
    plt.legend(loc='upper center')
    plt.show()