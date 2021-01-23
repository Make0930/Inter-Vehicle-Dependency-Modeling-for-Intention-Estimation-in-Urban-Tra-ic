import os
import processing_pipeline
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

    while True:
        random_trackid = random.randint(1, len(track_dictionary))
        if random_trackid in track_dictionary:
            print('trackid %s is found' % (random_trackid))
            break

    #random_trackid = 7
    print(random_trackid)

    if visualize:
        fig, axes = plt.subplots(1, 1)
        fig.canvas.set_window_title("Prediction Visualization")
        drawing_utils.draw_fancy_lanelet_map(laneletmap, axes)
        drawing_utils.draw_critical_areas(criticalAreas, axes)
        title_text = fig.suptitle("")
        fig2, axes2 = plt.subplots(2,1)
        # axes2.set_title("Data Visualization for random track: ",random_trackid)
        fig2.canvas.set_window_title("Data Visualization for track %s " % (random_trackid))
        plt.sca(axes2[0])
        plt.title('Absolute orientaion for all paths')
        plt.xlim(track_dictionary[random_trackid].time_stamp_ms_first,
                 track_dictionary[random_trackid].time_stamp_ms_last)
        plt.ylim(-3.2, 3.2)
        plt.sca(axes2[1])
        plt.title('Orientation difference for all paths')
        plt.xlim(track_dictionary[random_trackid].time_stamp_ms_first,
                 track_dictionary[random_trackid].time_stamp_ms_last)
        plt.ylim(-3.2, 3.2)
        plt.plot([track_dictionary[random_trackid].time_stamp_ms_first,track_dictionary[random_trackid].time_stamp_ms_last],[0,0],c = 'k')
        colors = ['g', 'b', 'c', 'm', 'y', 'k', 'orange', 'aqua', 'lime']
        markers = ['x', '+', 'v', '^', '1', '2', '3', '4', '5']
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
                possiblePathsWithInfo = []
                matchings = prediction_utilities.matchMotionState(laneletmap,currentMs)  # match the car to several possible lanelets
                for match in matchings:  # for each start lanelet
                    possiblePathParams.routingCostLimit = lanelet2.geometry.approximatedLength2d(match.lanelet) + 150
                    paths = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),
                                # caDict means conflict
                                graph.possiblePaths(match.lanelet, possiblePathParams))
                    possiblePathsWithInfo.extend(paths)
                vehicleState.pathsWithInformation = possiblePathsWithInfo
                activeObjects[track.track_id] = vehicleState
            else:
                vehicleState = activeObjects[track.track_id]
            vehicleState.update(currentMs)

        prediction_utilities.removeInactiveObjects(activeObjects, timestamp)

        # TODO: continue here - calculate matching, build lanelet->critical area dictionary, associate track -> next ca, estimate state

        if visualize:
            plt.sca(axes)
            # plt.axis('off')
            # drawing_utils.draw_motion_states(track_dictionary, timestamp, axes, patchesDict, textDict)
            drawing_utils.draw_vehicle_states(activeObjects, axes, patchesDict, textDict)
            prediction_utilities.cleanDrawingDicts(activeObjects, patchesDict, textDict)
            # fig.canvas.draw()
            title_text.set_text("\nts = {}".format(timestamp))
            if random_trackid in activeObjects.keys():
                basic_point = lanelet2.core.BasicPoint2d(
                    track_dictionary[random_trackid].motion_states[timestamp].x,
                    track_dictionary[random_trackid].motion_states[timestamp].y)
                in_area = 0
                for i in range(len(criticalAreas.critical_areas)):
                    area_center = lanelet2.core.BasicPoint2d(criticalAreas.critical_areas[i].x,criticalAreas.critical_areas[i].y)
                    if lanelet2.geometry.distance(basic_point,area_center) <= criticalAreas.critical_areas[i].radius:
                        plt.sca(axes2[0])
                        plt.scatter(timestamp, track_dictionary[random_trackid].motion_states[timestamp].psi_rad, c='k',
                                    s=10,label= 'vehicle %s in critical area' % random_trackid)
                        plt.sca(axes2[1])
                        plt.scatter(timestamp, track_dictionary[random_trackid].motion_states[timestamp].psi_rad, c='k',
                                    s=10,label= 'vehicle %s in critical area' % random_trackid)
                        in_area = 1
                        break
                if in_area == 0:
                    plt.sca(axes2[0])
                    plt.scatter(timestamp, track_dictionary[random_trackid].motion_states[timestamp].psi_rad, c='r',
                                s=1,label='vehicle %s orientation' % random_trackid)
                    plt.sca(axes2[1])
                    plt.scatter(timestamp, track_dictionary[random_trackid].motion_states[timestamp].psi_rad, c='r',
                                s=1,label='vehicle %s orientation' % random_trackid)

                for i in range(len(activeObjects[random_trackid].pathsWithInformation)):  # for each path
                    if (track_dictionary[random_trackid].motion_states[timestamp].psi_rad * activeObjects[random_trackid].pathOrientation[i][1] ) < 0 \
                            and (abs(activeObjects[random_trackid].pathOrientation[i][1]) + abs(track_dictionary[random_trackid].motion_states[timestamp].psi_rad)) > math.pi:
                        if activeObjects[random_trackid].pathOrientation[i][1] > 0:
                            arc_difference = 2* math.pi -  activeObjects[random_trackid].pathOrientation[i][1] \
                                             + track_dictionary[random_trackid].motion_states[timestamp].psi_rad
                        else:
                            arc_difference = 2* math.pi +  activeObjects[random_trackid].pathOrientation[i][1] \
                                             - track_dictionary[random_trackid].motion_states[timestamp].psi_rad
                    else:
                        arc_difference = abs(activeObjects[random_trackid].pathOrientation[i][1]
                                             - track_dictionary[random_trackid].motion_states[timestamp].psi_rad)
                    plt.sca(axes2[0])
                    plt.scatter(timestamp, activeObjects[random_trackid].pathOrientation[i][1] ,
                                    c=colors[i], s=10, label='path orientation %s' % (i),marker=markers[i])
                    plt.sca(axes2[1])
                    plt.scatter(timestamp, arc_difference, c=colors[i],s=10, label='path orientation %s' % (i),marker=markers[i])


            end_time = time.time()
            if timestamp == track_dictionary[random_trackid].time_stamp_ms_first:
                plt.legend()
            plt.pause(max(0.001, timestamp_delta_ms / 1000. - (end_time - start_time)))

        timestamp += timestamp_delta_ms
    if visualize:
        plt.ioff()
    return


if __name__=="__main__":
    main()