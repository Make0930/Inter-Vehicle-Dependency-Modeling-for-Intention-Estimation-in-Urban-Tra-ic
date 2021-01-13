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
import cmath


def startProcessingPipeline(args):
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

    laneletlength = dict()
    print(len(laneletmap.laneletLayer))
    for lanelet in laneletmap.laneletLayer:
        print(lanelet)
        print(lanelet.centerline[0])
        print(lanelet.centerline[-1])
        laneletlength[lanelet.id] = math.sqrt((lanelet.centerline[0].x - lanelet.centerline[-1].x )**2 +
                                               (lanelet.centerline[0].y - lanelet.centerline[-1].y) ** 2)

    # laneletorientation = {}
    # print(len(laneletmap.laneletLayer))
    # for lanelet in laneletmap.laneletLayer:
    #     print(lanelet)
    #     print(lanelet.centerline[0])
    #     print(lanelet.centerline[-1])
    #     coordinate =complex(lanelet.centerline[-1].x - lanelet.centerline[0].x, lanelet.centerline[-1].y - lanelet.centerline[0].y)
    #     laneletorientation[lanelet.id] = cmath.polar(coordinate)[1]



    visualize = args['visualize']

    #random select a track from track_dictionary
    while True:
        random_trackid = random.randint(1, len(track_dictionary))
        if random_trackid in track_dictionary:
            print('trackid %s is found' %(random_trackid))
            break

    #random_trackid = 16
    print(random_trackid)

    if visualize:
        fig, axes = plt.subplots(1, 1)
        # fig = plt.figure(1)
        # axes = plt.subplot(2, 1,1)
        # axes2 = plt.subplot(2,1,2)
        fig.canvas.set_window_title("Prediction Visualization")
        drawing_utils.draw_fancy_lanelet_map(laneletmap, axes)
        drawing_utils.draw_critical_areas(criticalAreas, axes)
        title_text = fig.suptitle("")
        fig2, axes2 = plt.subplots(1, 1)
        #axes2.set_title("Data Visualization for random track: ",random_trackid)
        fig2.canvas.set_window_title("Data Visualization for track %s " %(random_trackid))
        plt.xlim(track_dictionary[random_trackid].time_stamp_ms_first,track_dictionary[random_trackid].time_stamp_ms_last)
        plt.ylim(-3.2,3.2)
        colors = ['k', 'b', 'g', 'm','c', 'y']
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
                matchings = prediction_utilities.matchMotionState(laneletmap, currentMs)  # match the car to several possible lanelets
                possiblePathsWithInfo = []
                for match in matchings:      #for each start lanelet
                    possiblePathParams.routingCostLimit = lanelet2.geometry.approximatedLength2d(match.lanelet) + 150
                    paths = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),   #caDict means conflict
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
            #plt.axis('off')
            #drawing_utils.draw_motion_states(track_dictionary, timestamp, axes, patchesDict, textDict)
            drawing_utils.draw_vehicle_states(activeObjects, axes, patchesDict, textDict)
            prediction_utilities.cleanDrawingDicts(activeObjects, patchesDict, textDict)
            #fig.canvas.draw()
            title_text.set_text("\nts = {}".format(timestamp))
            plt.sca(axes2)
            if random_trackid in activeObjects.keys():
                #print(activeObjects[random_trackid].arcCoordinatesAlongPaths[0])
                #print('timestamp:',timestamp, 'length along path_0: ',activeObjects[random_trackid].arcCoordinatesAlongPaths[0].length)
                #print('timestamp:', timestamp, 'distance along path_0: ',
                #      activeObjects[random_trackid].arcCoordinatesAlongPaths[0].distance)

                #print(len(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence)) #all the lanelets in the first path
                # print(len(activeObjects[random_trackid].pathsWithInformation[0].centerline))  #all the center points in the first path
                # print(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0])   # the first lanelet in the first path
                # print(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline)  #all the center points in the first lanelet
                # print(len(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline))
                # print(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[0])   #first center point in the first lanelet
                # print(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[-1])  # last center point in the first lanelet
                # print(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[0])
                # print(cmath.sqrt((activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[0].x -
                #                   activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[-1].x) ** 2
                #                  + (activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[0].y -
                #                   activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[0].centerline[-1].y) ** 2 ))

                plt.scatter(timestamp,track_dictionary[random_trackid].motion_states[timestamp].psi_rad,c='r', s=1, label='vehicle %s orientation' %random_trackid)

                basicPoint = lanelet2.core.BasicPoint2d(track_dictionary[random_trackid].motion_states[timestamp].x,
                                                        track_dictionary[random_trackid].motion_states[timestamp].y)
                for i in range(len(activeObjects[random_trackid].pathsWithInformation)):    #for each path
                    arcCoordinates = lanelet2.geometry.toArcCoordinates(activeObjects[random_trackid].pathsWithInformation[i].centerline, basicPoint)
                    pathOrientation = activeObjects[random_trackid].pathsWithInformation[i].getOrientationAtArcLength(arcCoordinates.length)
                    plt.scatter(timestamp, pathOrientation, c=colors[i], s=10, label='path orientation %s' % (i), marker= markers[i])

                    # centerpoint_distance = []
                    # for j in range(len(activeObjects[random_trackid].pathsWithInformation[i].centerline)):    #for each point in the path
                    #     centerpoint_distance.append('%.4f' %math.sqrt((activeObjects[random_trackid].pathsWithInformation[i].centerline[j].x -
                    #               activeObjects[random_trackid].currentState.motionState.x) ** 2
                    #              + (activeObjects[random_trackid].pathsWithInformation[i].centerline[j].y -
                    #               activeObjects[random_trackid].currentState.motionState.y) ** 2 ))
                    # #print(len(centerpoint_distance))
                    # #print(min(centerpoint_distance))
                    # point_id = centerpoint_distance.index(min(centerpoint_distance))
                    # searched_id_sum = 0
                    # lanelet_order = 0
                    # for k in range(len(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence)):  # k = lanelet order in the path
                    #     #print(len(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[k].centerline))
                    #     if (searched_id_sum + len(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[k].centerline)) >= point_id:
                    #         lanelet_order = k
                    #         break
                    #     searched_id_sum += len(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[k].centerline)
                    #print(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[lanelet_order])
                    # coordinate = complex(activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[lanelet_order].centerline[-1].x
                    #                      - activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[lanelet_order].centerline[0].x,
                    #                      activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[lanelet_order].centerline[-1].y
                    #                      - activeObjects[random_trackid].pathsWithInformation[i].laneletSequence[lanelet_order].centerline[0].y)
                    # coordinate = complex(activeObjects[random_trackid].pathsWithInformation[i].centerline[point_id].x
                    #                      - activeObjects[random_trackid].pathsWithInformation[i].centerline[point_id - 1].x,
                    #                      activeObjects[random_trackid].pathsWithInformation[i].centerline[point_id].y
                    #                      - activeObjects[random_trackid].pathsWithInformation[i].centerline[point_id - 1].y)
                    # laneletorientation = cmath.polar(coordinate)[1]
                    # #print(laneletorientation)
                    # #print(track_dictionary[random_trackid].motion_states[timestamp].psi_rad)
                    # plt.scatter(timestamp, laneletorientation, c=colors[i], s=2,label='path orientation %s' %(i))

                #test the matching of path centerline number and the sum of lanelet centerline number
                # print(len(activeObjects[random_trackid].pathsWithInformation[0].centerline))
                # s = 0
                # for i in range(len(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence)):
                #     #print(len(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[i].centerline))
                #     s += len(activeObjects[random_trackid].pathsWithInformation[0].laneletSequence[i].centerline)
                # print(s)
            end_time = time.time()
            if timestamp == track_dictionary[random_trackid].time_stamp_ms_first:
                plt.legend()
            plt.pause(max(0.001, timestamp_delta_ms / 1000. - (end_time - start_time)))
        timestamp += timestamp_delta_ms
    if visualize:
        plt.ioff()
    return
