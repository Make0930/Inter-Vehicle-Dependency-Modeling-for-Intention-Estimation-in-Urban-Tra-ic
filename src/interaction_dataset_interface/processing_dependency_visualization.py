import os
import processing_pipeline_edit
import threading # only for plots
import map_analyzer
import lanelet2
import lanelet2.core
import lanelet2.geometry
import lanelet2.routing
import dataset_reader
import interaction_dataset_utilities
import drawing_utils
import matplotlib.pyplot as plt
import time
import prediction_utilities
import predictiontypes
from lanelet2.core import LaneletSequence
from lanelet2_matching import python
import random
import math
import networkx as nx


if __name__=='__main__' and __package__ is None:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')

def main() :
    from argparser import parse_arguments
    args = parse_arguments(ROOT_PATH)
    print(ROOT_PATH)
    print('loading map...')
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args['lat_origin'], args['lon_origin']))
    laneletmap = lanelet2.io.load(args['lanelet_map'], projector)
    trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                 lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(laneletmap, trafficRules)

    print('analyzing map...')
    criticalAreas = map_analyzer.getAllCriticalAreas(laneletmap, graph, args['critical_area_sim_thresh'])
    laneletCaDict = map_analyzer.createLaneletCriticalAreaDict(criticalAreas)

    print('loading trackfiles')
    track_dictionary = dataset_reader.read_tracks(args['interaction_trackfile'])

    timestamp_min = 1e9
    timestamp_max = 0
    timestamp_delta_ms = 100
    for key, track in iter(track_dictionary.items()):
        timestamp_min = min(timestamp_min, track.time_stamp_ms_first)
        timestamp_max = max(timestamp_max, track.time_stamp_ms_last)
    timestamp = timestamp_min
    patchesDict = dict()
    textDict = dict()


    visualize = args['visualize']


    # FOR ROUNDABOUT MAP
    roundabout_id = {30001, 30002, 30005, 30016, 30018, 30030, 30036, 30040, 30042, 30047, 30004, 30017, 30023,30034,30038}
    in_top_left_id = {30006, 30025, 30026, 30027, 30015}
    in_bottom_id = {30031, 30033, 30039, 30043, 30000}
    in_top_right_id = {30029, 30021, 30014, 30012, 30010, 30046}
    out_top_left_id = {30022, 30024, 30007, 30008, 30045, 30032}
    out_bottom_id ={30037, 30035, 30041, 30044, 30019}
    out_top_right_id ={30028, 30020, 30013, 30011, 30009, 30003}
    left_area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[0].x, criticalAreas.critical_areas[0].y)
    bottom_area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[4].x,criticalAreas.critical_areas[4].y)
    right_area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[3].x,criticalAreas.critical_areas[3].y)
    conflict_left_area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[2].x,criticalAreas.critical_areas[2].y)
    conflict_right_area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[1].x,criticalAreas.critical_areas[1].y)

    if visualize:
        fig, axes = plt.subplots(1, 1)
        fig.canvas.set_window_title("Prediction Visualization")
        drawing_utils.draw_fancy_lanelet_map(laneletmap, axes)
        drawing_utils.draw_critical_areas(criticalAreas, axes)
        title_text = fig.suptitle("")
        fig2, axes2 = plt.subplots(1, 1)
        fig2.canvas.set_window_title("Real Time Dependency Visualization")
        G = nx.DiGraph()
        plt.ion()
        plt.show()

    activeObjects = dict()
    while timestamp < timestamp_max:
        if visualize:
            start_time = time.time()

        currentTracks = interaction_dataset_utilities.getVisibleTracks(timestamp, track_dictionary)

        possiblePathParams = lanelet2.routing.PossiblePathsParams()
        possiblePathParams.includeShorterPaths = True
        possiblePathParams.includeLaneChanges = False
        for track in currentTracks:
            currentMs = track.motion_states[timestamp]
            if track.track_id not in activeObjects:
                vehicleState = predictiontypes.Vehicle(objectId=track.track_id, motionState=currentMs,
                                                       width=track.width, length=track.length)
                matchings = prediction_utilities.matchMotionState(laneletmap,
                                                                  currentMs)  # match the car to several possible lanelets
                possiblePathsWithInfo = []
                for match in matchings:  # for each start lanelet
                    possiblePathParams.routingCostLimit = lanelet2.geometry.approximatedLength2d(match.lanelet) + 150
                    paths = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),
                                # caDict means conflict
                                graph.possiblePaths(match.lanelet, possiblePathParams))
                    possiblePathsWithInfo.extend(paths)
                vehicleState.pathsWithInformation = possiblePathsWithInfo
                vehicleState.laneletMatchings = matchings
                activeObjects[track.track_id] = vehicleState
            else:
                vehicleState = activeObjects[track.track_id]
                matchings = prediction_utilities.matchMotionState(laneletmap,currentMs)  # match the car to several possible lanelets
                vehicleState.laneletMatchings = matchings
                #activeObjects[track.track_id] = vehicleState

            vehicleState.update(currentMs)

        prediction_utilities.removeInactiveObjects(activeObjects, timestamp)

        # TODO: continue here - calculate matching, build lanelet->critical area dictionary, associate track -> next ca, estimate state

        if visualize:
            plt.sca(axes)
            # plt.axis('off')
            # drawing_utils.draw_motion_states(track_dictionary, timestamp, axes, patchesDict, textDict)
            drawing_utils.draw_vehicle_states(activeObjects, axes, patchesDict, textDict)
            prediction_utilities.cleanDrawingDicts(activeObjects, patchesDict, textDict)
            title_text.set_text("\nts = {}".format(timestamp))
            #print(timestamp)

            roundabout_car = []    # the id of cars
            in_top_left_car = []
            in_bottom_car = []
            in_top_right_car = []
            out_top_left_car = []
            out_top_right_car = []
            out_bottom_car = []
            criticalarea_left = []
            criticalarea_right = []
            criticalarea_bottom = []
            for key, value in activeObjects.items():      #for each car    key= car id
                for i in value.laneletMatchings:          #for each possible lanelet id of a car
                    if i.lanelet.id in roundabout_id:
                        #print('vehicle ', key, ' is in roundabout', ' and lanelet_id is ',i.lanelet.id )
                        roundabout_car.append(key)
                        break  #if car 1 in roundabout, skip the following judgement
                    #print(key, i.lanelet.id)
            for key, value in activeObjects.items():
                if key in roundabout_car:
                    continue
                else:
                    for i in value.laneletMatchings:
                        if i.lanelet.id in in_top_left_id:
                            in_top_left_car.append(key)
                            break
                        elif i.lanelet.id in in_bottom_id:
                            in_bottom_car.append(key)
                            break
                        elif i.lanelet.id in in_top_right_id:
                            in_top_right_car.append(key)
                            break
                        elif i.lanelet.id in out_top_left_id:
                            out_top_left_car.append(key)
                            break
                        elif i.lanelet.id in out_top_right_id:
                            out_top_right_car.append(key)
                            break
                        elif i.lanelet.id in out_bottom_id:
                            out_bottom_car.append(key)
                            break

            roundabout_dependency_edges = []
            right_dependency_edges = []
            left_dependency_edges = []
            bottom_dependency_edges = []
            right_roundabout_edges = []
            left_roundabout_edges = []
            bottom_roundabout_edges = []

            dependency_nodes = roundabout_car + in_top_left_car + in_bottom_car + in_top_right_car + out_top_left_car + out_bottom_car + out_top_right_car
            if roundabout_car != []:
                for i in roundabout_car:  # for each center car, check weather the car is in which critial area         id = i
                    basic_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[i].motion_states[timestamp].x,
                        track_dictionary[i].motion_states[timestamp].y)
                    if lanelet2.geometry.distance(basic_point, left_area_center) < criticalAreas.critical_areas[0].radius:
                        criticalarea_left.append(i)
                        continue
                    elif lanelet2.geometry.distance(basic_point, bottom_area_center) < criticalAreas.critical_areas[4].radius:
                        criticalarea_bottom.append(i)
                        continue
                    elif lanelet2.geometry.distance(basic_point, right_area_center) < criticalAreas.critical_areas[3].radius:
                        criticalarea_right.append(i)

            #roundabout sequence
            if len(roundabout_car) > 1:
                #print('roundabout_car:', roundabout_car)
                calculated_pair = {}
                for i in range(len(roundabout_car)):  #for each center car, check weather the car is in which critial area other car is in its possible paths   id = roundabout_car[i]
                   # print('center car number:', roundabout_car[i])
                    basic_point = lanelet2.core.BasicPoint2d(track_dictionary[roundabout_car[i]].motion_states[timestamp].x,
                                                             track_dictionary[roundabout_car[i]].motion_states[timestamp].y)
                    for j in roundabout_car:  # for each other car   id = j
                        if j == roundabout_car[i] or (roundabout_car[i] in calculated_pair and j in calculated_pair[roundabout_car[i]]): #skip, if center car ==  other car or center car and other car are already paird
                            continue
                        the_car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                                   track_dictionary[j].motion_states[timestamp].y)
                        for k in range(
                                len(activeObjects[roundabout_car[i]].pathsWithInformation)):  # for each possible path, but need update

                            arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[roundabout_car[i]].pathsWithInformation[k].centerline, basic_point)
                           # print(arcCoordinates.length)
                            car_arcCoordinates = lanelet2.geometry.toArcCoordinates(activeObjects[roundabout_car[i]].pathsWithInformation[k].centerline, the_car_point)

                            if abs(car_arcCoordinates.distance) < 4.5 and car_arcCoordinates.length > arcCoordinates.length and (car_arcCoordinates.length - arcCoordinates.length) < 18:
                                roundabout_dependency_edges.append((j, roundabout_car[i]))
                                calculated_pair.setdefault(j,[]).append(roundabout_car[i])
                                break

            # right_car_sequence
            if len(in_top_right_car) > 1:
               # print('in_top_right_car ',in_top_right_car)
                calculated_distance = []
                for i in range(len(in_top_right_car)):  #for each center car, check weather the car is in which critial area other car is in its possible paths   id = roundabout_car[i]
                    basic_point = lanelet2.core.BasicPoint2d(track_dictionary[in_top_right_car[i]].motion_states[timestamp].x,
                                                             track_dictionary[in_top_right_car[i]].motion_states[timestamp].y)
                    calculated_distance.append(lanelet2.geometry.distance(left_area_center,basic_point))
                sorted_distance = sorted(range(len(calculated_distance)),key= lambda k: calculated_distance[k], reverse= False)  #from small to large distance
                for j in range(len(sorted_distance)-1):   # car id = in_top_right_car[sorted_distance[j]]
                    right_dependency_edges.append((in_top_right_car[sorted_distance[j]],in_top_right_car[sorted_distance[j + 1]]))

            #left car sequence
            if len(in_top_left_car) > 1:
               # print('in_top_right_car ',in_top_right_car)
                calculated_distance = []
                for i in range(len(in_top_left_car)):  #for each center car, check weather the car is in which critial area other car is in its possible paths   id = roundabout_car[i]
                    basic_point = lanelet2.core.BasicPoint2d(track_dictionary[in_top_left_car[i]].motion_states[timestamp].x,
                                                             track_dictionary[in_top_left_car[i]].motion_states[timestamp].y)
                    calculated_distance.append((lanelet2.geometry.distance(bottom_area_center,basic_point)))
                sorted_distance = sorted(range(len(calculated_distance)),key= lambda k: calculated_distance[k], reverse= False)  #from small to large distance
                for j in range(len(sorted_distance)-1):   # car id = in_top_right_car[sorted_distance[j]]
                    left_dependency_edges.append((in_top_left_car[sorted_distance[j]],in_top_left_car[sorted_distance[j + 1]]))

            #bottom car sequence
            if len(in_bottom_car) > 1:
               # print('in_top_right_car ',in_top_right_car)
                calculated_distance = []
                for i in range(len(in_bottom_car)):  #for each center car, check weather the car is in which critial area other car is in its possible paths   id = roundabout_car[i]
                    basic_point = lanelet2.core.BasicPoint2d(track_dictionary[in_bottom_car[i]].motion_states[timestamp].x,
                                                             track_dictionary[in_bottom_car[i]].motion_states[timestamp].y)
                    calculated_distance.append((lanelet2.geometry.distance(right_area_center,basic_point)))
                sorted_distance = sorted(range(len(calculated_distance)),key= lambda k: calculated_distance[k], reverse= False)  #from small to large distance
                for j in range(len(sorted_distance)-1):   # car id = in_top_right_car[sorted_distance[j]]
                    left_dependency_edges.append((in_bottom_car[sorted_distance[j]],in_bottom_car[sorted_distance[j + 1]]))


            #right-in-roundabout interaction
            if roundabout_car != [] and in_top_right_car != []:
                distance_basic_right = []
                for j in in_top_right_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                           track_dictionary[j].motion_states[timestamp].y)
                    distance_basic_right.append(lanelet2.geometry.distance(conflict_right_area_center, car_point))
                min_distance = min(distance_basic_right)
                if min_distance < 36:   #the closest car's distance to the right critical area center
                    min_index = distance_basic_right.index(min_distance)  #the first car index in right line
                    right_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[in_top_right_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[in_top_right_car[min_index]].motion_states[timestamp].y)
                    for i in roundabout_car:  #dependency from roundabout to right line
                        roundabout_car_point = lanelet2.core.BasicPoint2d(
                            track_dictionary[i].motion_states[timestamp].x,
                            track_dictionary[i].motion_states[timestamp].y)
                        distance = lanelet2.geometry.distance(right_car_point, roundabout_car_point)
                        distance2 = lanelet2.geometry.distance(conflict_right_area_center,roundabout_car_point)
                        if distance2 < min_distance +5 and distance < 25:
                            right_roundabout_edges.append((i,in_top_right_car[min_index]))
                    if criticalarea_right != []:
                        for k in criticalarea_right:
                            critical_car_point = lanelet2.core.BasicPoint2d(
                                track_dictionary[k].motion_states[timestamp].x,
                                track_dictionary[k].motion_states[timestamp].y)
                            critial_distance = lanelet2.geometry.distance(critical_car_point,conflict_right_area_center)
                            if critial_distance > min_distance:
                                right_roundabout_edges.append((in_top_right_car[min_index],k))

            #left-in-roundabout-interaction
            if roundabout_car != [] and in_top_left_car != []:
                distance_basic_right = []
                for j in in_top_left_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                           track_dictionary[j].motion_states[timestamp].y)
                    distance_basic_right.append(lanelet2.geometry.distance(conflict_left_area_center, car_point))
                min_distance = min(distance_basic_right)
                if min_distance < 35:   #the closest car's distance to the right critical area center
                    min_index = distance_basic_right.index(min_distance)  #the first car index in left line
                    left_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[in_top_left_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[in_top_left_car[min_index]].motion_states[timestamp].y)
                    for i in roundabout_car:  #dependency from roundabout to right line
                        roundabout_car_point = lanelet2.core.BasicPoint2d(
                            track_dictionary[i].motion_states[timestamp].x,
                            track_dictionary[i].motion_states[timestamp].y)
                        distance = lanelet2.geometry.distance(left_car_point, roundabout_car_point)  #distance between the cloest left car  and each roundabout car
                        distance2 = lanelet2.geometry.distance(conflict_left_area_center,roundabout_car_point)
                        if distance2 < min_distance +5 and distance < 25:
                            left_roundabout_edges.append((i,in_top_left_car[min_index]))
                    if criticalarea_left != []:
                        for k in criticalarea_left:
                            critical_car_point = lanelet2.core.BasicPoint2d(
                                track_dictionary[k].motion_states[timestamp].x,
                                track_dictionary[k].motion_states[timestamp].y)
                            critial_distance = lanelet2.geometry.distance(critical_car_point,conflict_left_area_center)
                            if critial_distance > min_distance:
                                left_roundabout_edges.append((in_top_left_car[min_index],k))

            #bottom-in-roundabout-interaction
            if roundabout_car != [] and in_bottom_car != []:
                distance_basic_right = []
                for j in in_bottom_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                           track_dictionary[j].motion_states[timestamp].y)
                    distance_basic_right.append(lanelet2.geometry.distance(right_area_center, car_point))
                min_distance = min(distance_basic_right)
                if min_distance < 30:   #the closest car's distance to the right critical area center
                    min_index = distance_basic_right.index(min_distance)  #the first car index in right line
                    bottom_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[in_bottom_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[in_bottom_car[min_index]].motion_states[timestamp].y)
                    for i in roundabout_car:  #dependency from roundabout to right line
                        roundabout_car_point = lanelet2.core.BasicPoint2d(
                            track_dictionary[i].motion_states[timestamp].x,
                            track_dictionary[i].motion_states[timestamp].y)
                        distance = lanelet2.geometry.distance(bottom_car_point, roundabout_car_point)
                        distance2 = lanelet2.geometry.distance(right_area_center,roundabout_car_point)
                        if distance2 < min_distance + 5 and distance < 25:
                            bottom_roundabout_edges.append((i,in_bottom_car[min_index]))
                    if criticalarea_bottom != []:
                        for k in criticalarea_bottom:
                            critical_car_point = lanelet2.core.BasicPoint2d(
                                track_dictionary[k].motion_states[timestamp].x,
                                track_dictionary[k].motion_states[timestamp].y)
                            critial_distance = lanelet2.geometry.distance(critical_car_point,right_area_center)
                            if critial_distance > min_distance:
                                bottom_roundabout_edges.append((in_bottom_car[min_index],k))

            #bottom-out-roundabout-interaction
            if roundabout_car != [] and out_bottom_car != []:
                distance_out_bottom = []
                for i in out_bottom_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[i].motion_states[timestamp].x,
                                                           track_dictionary[i].motion_states[timestamp].y)
                    distance_out_bottom.append(lanelet2.geometry.distance(bottom_area_center,car_point))
                min_distance = min(distance_out_bottom)
                if min_distance <25:
                    min_index = distance_out_bottom.index(min_distance)  #the last car index in out bottom line
                    bottom_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[out_bottom_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[out_bottom_car[min_index]].motion_states[timestamp].y
                    )
                    compare_distance = []
                    for j in roundabout_car:  #for each car in roundabout
                        the_car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                                   track_dictionary[j].motion_states[timestamp].y)
                        for k in range(len(activeObjects[j].pathsWithInformation)):  # for each possible path, but need update
                            arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, the_car_point)
                            # print(arcCoordinates.length)
                            car_arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, bottom_car_point)
                            # print('car ',j, ' arccoordinates.length is', car_arcCoordinates.length, ' arccoordinates.distance is', car_arcCoordinates.distance)
                            if abs(car_arcCoordinates.distance) < 3 and car_arcCoordinates.length > arcCoordinates.length and car_arcCoordinates.length - arcCoordinates.length < 40:
                                # print('car',in_top_right_car[i] , ' is behind of car', j,' with the distance: ',car_arcCoordinates.length - arcCoordinates.length)
                                compare_distance.append(car_arcCoordinates.length - arcCoordinates.length)
                                break
                    if compare_distance != []:
                        min_ind = compare_distance.index(min(compare_distance))
                        bottom_dependency_edges.append((out_bottom_car[min_index], roundabout_car[min_ind]))

            #left-out-roundabout-interaction
            if roundabout_car != [] and out_top_left_car != []:
                distance_out_left = []
                for i in out_top_left_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[i].motion_states[timestamp].x,
                                                           track_dictionary[i].motion_states[timestamp].y)
                    distance_out_left.append(lanelet2.geometry.distance(left_area_center,car_point))
                min_distance = min(distance_out_left)
                if min_distance <25:
                    min_index = distance_out_left.index(min_distance)  #the last car index in out left line
                    left_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[out_top_left_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[out_top_left_car[min_index]].motion_states[timestamp].y
                    )
                    compare_distance = []
                    for j in roundabout_car:  #for each car in roundabout
                        the_car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                                   track_dictionary[j].motion_states[timestamp].y)
                        for k in range(len(activeObjects[j].pathsWithInformation)):  # for each possible path, but need update
                            arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, the_car_point)
                            # print(arcCoordinates.length)
                            car_arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, left_car_point)
                            # print('car ',j, ' arccoordinates.length is', car_arcCoordinates.length, ' arccoordinates.distance is', car_arcCoordinates.distance)
                            if abs(car_arcCoordinates.distance) < 3 and car_arcCoordinates.length > arcCoordinates.length and car_arcCoordinates.length - arcCoordinates.length < 40:
                                # print('car',in_top_right_car[i] , ' is behind of car', j,' with the distance: ',car_arcCoordinates.length - arcCoordinates.length)
                                compare_distance.append(car_arcCoordinates.length - arcCoordinates.length)
                                break
                    if compare_distance != []:
                        min_ind = compare_distance.index(min(compare_distance))
                        left_dependency_edges.append((out_top_left_car[min_index], roundabout_car[min_ind]))

            #right-out-roundabout-interaction
            if roundabout_car != [] and out_top_right_car != []:
                distance_out_right = []
                for i in out_top_right_car:
                    car_point = lanelet2.core.BasicPoint2d(track_dictionary[i].motion_states[timestamp].x,
                                                           track_dictionary[i].motion_states[timestamp].y)
                    distance_out_right.append(lanelet2.geometry.distance(right_area_center,car_point))
                min_distance = min(distance_out_right)
                if min_distance <25:
                    min_index = distance_out_right.index(min_distance)  #the last car index in out bottom line
                    right_car_point = lanelet2.core.BasicPoint2d(
                        track_dictionary[out_top_right_car[min_index]].motion_states[timestamp].x,
                        track_dictionary[out_top_right_car[min_index]].motion_states[timestamp].y)
                    compare_distance = []
                    for j in roundabout_car:  #for each car in roundabout
                        the_car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                                   track_dictionary[j].motion_states[timestamp].y)
                        for k in range(len(activeObjects[j].pathsWithInformation)):  # for each possible path, but need update
                            arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, the_car_point)
                            # print(arcCoordinates.length)
                            car_arcCoordinates = lanelet2.geometry.toArcCoordinates(
                                activeObjects[j].pathsWithInformation[k].centerline, right_car_point)
                            # print('car ',j, ' arccoordinates.length is', car_arcCoordinates.length, ' arccoordinates.distance is', car_arcCoordinates.distance)
                            if abs(car_arcCoordinates.distance) < 3 and car_arcCoordinates.length > arcCoordinates.length and car_arcCoordinates.length - arcCoordinates.length < 40:
                                # print('car',in_top_right_car[i] , ' is behind of car', j,' with the distance: ',car_arcCoordinates.length - arcCoordinates.length)
                                compare_distance.append(car_arcCoordinates.length - arcCoordinates.length)
                                break
                    if compare_distance != []:
                        min_ind = compare_distance.index(min(compare_distance))
                        right_dependency_edges.append((out_top_right_car[min_index], roundabout_car[min_ind]))


            dependency_edges = roundabout_dependency_edges + left_dependency_edges + right_dependency_edges + bottom_dependency_edges + right_roundabout_edges + left_roundabout_edges + bottom_roundabout_edges

            plt.sca(axes2)
            G.clear()
            axes2.cla()
            G.add_edges_from(dependency_edges)
            G.add_nodes_from(dependency_nodes)
            nx.draw_networkx(G)
            end_time = time.time()
            #plt.pause(0.5)
            plt.pause(max(0.001, timestamp_delta_ms / 1000. - (end_time - start_time)))
        timestamp += timestamp_delta_ms
    if visualize:
        plt.ioff()
    return


if __name__=="__main__":
    main()