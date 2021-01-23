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


    visualize = args['visualize']

    if visualize:
        fig, axes = plt.subplots(1, 1)
        # fig = plt.figure(1)
        # axes = plt.subplot(2, 1,1)
        fig.canvas.set_window_title("Prediction Visualization")
        drawing_utils.draw_fancy_lanelet_map(laneletmap, axes)
        drawing_utils.draw_critical_areas(criticalAreas, axes)
        title_text = fig.suptitle("")
        #axes2.set_title("Data Visualization for random track: ",random_trackid)
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
            end_time = time.time()
            plt.pause(max(0.001, timestamp_delta_ms / 1000. - (end_time - start_time)))
        timestamp += timestamp_delta_ms
    if visualize:
        plt.ioff()
    return
