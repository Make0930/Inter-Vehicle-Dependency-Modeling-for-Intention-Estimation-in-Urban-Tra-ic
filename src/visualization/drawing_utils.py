import matplotlib
import matplotlib.axes
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import numpy as np
import predictiontypes


def set_visible_area(laneletmap, axes):
    min_x = 10e9
    min_y = 10e9
    max_x = -10e9
    max_y = -10e9

    for point in laneletmap.pointLayer:
        min_x = min(point.x, min_x)
        min_y = min(point.y, min_y)
        max_x = max(point.x, max_x)
        max_y = max(point.y, max_y)

    axes.set_aspect('equal', adjustable='box')
    axes.set_xlim([min_x - 10, max_x + 10])
    axes.set_ylim([min_y - 10, max_y + 10])


def draw_lanelet_map(laneletmap, axes):
    assert isinstance(axes, matplotlib.axes.Axes)
    set_visible_area(laneletmap, axes)

    added_label = True
    for ll in laneletmap.laneletLayer:
        if not added_label:
            plt.plot([p.x for p in ll.leftBound], [p.y for p in ll.leftBound], 'r--', label="left boundary")
            plt.plot([p.x for p in ll.rightBound], [p.y for p in ll.rightBound], 'g-.', label="right boundary")
            added_label = True
        else:
            plt.plot([p.x for p in ll.leftBound], [p.y for p in ll.leftBound], 'r--')
            plt.plot([p.x for p in ll.rightBound], [p.y for p in ll.rightBound], 'g-.')


def draw_fancy_lanelet_map(laneletmap, axes):
    set_visible_area(laneletmap, axes)
    unknown_linestring_types = list()

    for ls in laneletmap.lineStringLayer:
        if "type" not in ls.attributes.keys():
            raise RuntimeError("ID " + str(ls.id) + ": Linestring type must be specified")
        elif ls.attributes["type"] == "curbstone":
            type_dict = dict(color="black", linewidth=1, zorder=10)
        elif ls.attributes["type"] == "line_thin":
            if "subtype" in ls.attributes.keys() and ls.attributes["subtype"] == "dashed":
                type_dict = dict(color="white", linewidth=1, zorder=10, dashes=[10, 10])
            else:
                type_dict = dict(color="white", linewidth=1, zorder=10)
        elif ls.attributes["type"] == "line_thick":
            if "subtype" in ls.attributes.keys() and ls.attributes["subtype"] == "dashed":
                type_dict = dict(color="white", linewidth=2, zorder=10, dashes=[10, 10])
            else:
                type_dict = dict(color="white", linewidth=2, zorder=10)
        elif ls.attributes["type"] == "pedestrian_marking":
            type_dict = dict(color="white", linewidth=1, zorder=10, dashes=[5, 10])
        elif ls.attributes["type"] == "bike_marking":
            type_dict = dict(color="white", linewidth=1, zorder=10, dashes=[5, 10])
        elif ls.attributes["type"] == "stop_line":
            type_dict = dict(color="white", linewidth=3, zorder=10)
        elif ls.attributes["type"] == "virtual":
            type_dict = dict(color="blue", linewidth=1, zorder=10, dashes=[2, 5])
        elif ls.attributes["type"] == "road_border":
            type_dict = dict(color="black", linewidth=1, zorder=10)
        elif ls.attributes["type"] == "guard_rail":
            type_dict = dict(color="black", linewidth=1, zorder=10)
        elif ls.attributes["type"] == "traffic_sign":
            continue
        else:
            if ls.attributes["type"] not in unknown_linestring_types:
                unknown_linestring_types.append(ls.attributes["type"])
            continue
        ls_points_x = [pt.x for pt in ls]
        ls_points_y = [pt.y for pt in ls]
        plt.plot(ls_points_x, ls_points_y, **type_dict)

    if len(unknown_linestring_types) != 0:
        print("Found the following unknown types, did not plot them: " + str(unknown_linestring_types))

    lanelets = []
    for ll in laneletmap.laneletLayer:
        points = [[pt.x, pt.y] for pt in ll.polygon2d()]
        polygon = Polygon(points, True)
        lanelets.append(polygon)

    ll_patches = PatchCollection(lanelets, facecolors="lightgray", edgecolors="None", zorder=5)
    axes.add_collection(ll_patches)
    if len(laneletmap.laneletLayer) == 0:
        axes.patch.set_facecolor('lightgrey')
    areas = []
    for area in laneletmap.areaLayer:
        if area.attributes["subtype"] == "keepout":
            points = [[pt.x, pt.y] for pt in area.outerBoundPolygon()]
            polygon = Polygon(points, True)
            areas.append(polygon)
    area_patches = PatchCollection(areas, facecolors="darkgray", edgecolors="None", zorder=5)
    axes.add_collection(area_patches)


def draw_critical_areas(criticalAreas, axes):
    for ca in criticalAreas:
        assert isinstance(ca, predictiontypes.CriticalArea)
        if ca.description == "diverging":
            circle = plt.Circle((ca.x, ca.y), ca.radius, facecolor='g', edgecolor="black", zorder=15)
        elif ca.description == "conflicting":
            circle = plt.Circle((ca.x, ca.y), ca.radius, facecolor='r', edgecolor="black", zorder=15)
        else:          #unified(both)
            circle = plt.Circle((ca.x, ca.y), ca.radius, facecolor='yellow', edgecolor="black", zorder=15)
        axes.add_artist(circle)
        # plt.gca().add_artist(circle)


def draw_diverging_points(divergingPoints, axes):
    for dp in divergingPoints:
        circle = plt.Circle((dp.x, dp.y), 2, facecolor='g', edgecolor="black", zorder=17)
        axes.add_artist(circle)


def polygon_xy_from_motionstate(ms, width, length):
    lowleft = (ms.x - length / 2., ms.y - width / 2.)
    lowright = (ms.x + length / 2., ms.y - width / 2.)
    upright = (ms.x + length / 2., ms.y + width / 2.)
    upleft = (ms.x - length / 2., ms.y + width / 2.)
    #pts are the coordinate of four corners of a car shape
    pts = np.array([lowleft, lowright, upright, upleft])
    center = np.array([ms.x, ms.y])
    yaw = ms.psi_rad
    return np.dot(pts - center, np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])) + center


def draw_motion_state(ms, width, length, id, patchesDict, textDict, axes, color=None):
    if id not in patchesDict:
        polygon = polygon_xy_from_motionstate(ms, width, length)
        rect = matplotlib.patches.Polygon(polygon, closed=True, zorder=20, facecolor=color, edgecolor='black')
        patchesDict[id] = rect
        axes.add_patch(rect)
        textDict[id] = axes.text(ms.x, ms.y + 2, str(id), horizontalalignment='center', zorder=30)
    else:
        patchesDict[id].set_xy(polygon_xy_from_motionstate(ms, width, length))
        textDict[id].set_position((ms.x, ms.y + 2))

# def draw_motion_state(ms, width, length, id, patchesDict, textDict, axes, color=None):
#
#     polygon = polygon_xy_from_motionstate(ms, width, length)
#     rect = matplotlib.patches.Polygon(polygon, closed=True, zorder=20, facecolor=color, edgecolor='black')
#     patchesDict[id] = rect
#     axes.add_patch(rect)
#     textDict[id] = axes.text(ms.x, ms.y + 2, str(id), horizontalalignment='center', zorder=30)



def draw_motion_states(trackDict, timestamp, axes, patchesDict, textDict):
    for key, value in trackDict.items():
        if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last:
            width = value.width
            length = value.length
            ms = value.motion_states[timestamp]
            draw_motion_state(ms=ms, width=value.width, length=value.length, id=key, patchesDict=patchesDict,
                              textDict=textDict, axes=axes)


def draw_vehicle_states(activeObjects, axes, patchesDict, textDict):
    for id, vehicle in activeObjects.items():
        draw_motion_state(ms=vehicle.currentState.motionState, width=vehicle.width, length=vehicle.length, id=id,
                          patchesDict=patchesDict, textDict=textDict, axes=axes, color=vehicle.color)
