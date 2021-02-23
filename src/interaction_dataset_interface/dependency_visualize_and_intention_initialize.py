import os
import processing_pipeline
# import threading # only for plots
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
import networkx as nx
from possiblepath_calculation import possiblepath_calculate
from dependency_modeling import dependency_calculate
from hidden_intention_initialization import hidden_intentions_initialize


if __name__=='__main__' and __package__ is None:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')

def main() :
    from argparser import parse_arguments
    args = parse_arguments(ROOT_PATH)
    print(ROOT_PATH)
    print('loading map...')
    map_name = (args['lanelet_map'].split('/')[-1]).split('.')[0]
    trackfile = (args['interaction_trackfile'].split('/')[-1]).split('_')[-1]
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

    #parameter setting
    visualize = args['visualize']
    Deviation_alongarcCoordinate = 4
    Time_difference_max = 3
    Time_gap = 5
    Default_gap = 40
    Distance_difference_max = 20
    PathLengthLimit = 500

    if visualize:
        fig, axes = plt.subplots(1, 1)
        fig.canvas.set_window_title("Prediction Visualization in %s for %s" %(map_name,trackfile))
        drawing_utils.draw_fancy_lanelet_map(laneletmap, axes)
        drawing_utils.draw_critical_areas(criticalAreas, axes)
        title_text = fig.suptitle("")
        fig2, axes2 = plt.subplots(1, 1)
        fig2.canvas.set_window_title("Dependency Visualization in %s for %s" %(map_name,trackfile))
        G = nx.DiGraph()
        plt.ion()
        plt.show()


    activeObjects = dict()
    while timestamp < timestamp_max:
        if visualize:
            start_time = time.time()

        currentTracks = interaction_dataset_utilities.getVisibleTracks(timestamp, track_dictionary)

        # possiblePathParams = lanelet2.routing.PossiblePathsParams()
        # possiblePathParams.includeShorterPaths = True
        # possiblePathParams.includeLaneChanges = False
        for track in currentTracks:
            currentMs = track.motion_states[timestamp]
            if track.track_id not in activeObjects:
                vehicleState = predictiontypes.Vehicle(objectId=track.track_id, motionState=currentMs,
                                                       width=track.width, length=track.length, timestamp_first= track.time_stamp_ms_first)
                possiblePathsWithInfo = []
                matchings = prediction_utilities.matchMotionState(laneletmap,currentMs)  # match the car to several possible lanelets
                for match in matchings:  # for each start lanelet
                    #possiblePathParams.routingCostLimit = lanelet2.geometry.approximatedLength2d(match.lanelet) + 300   # a very important value. it means how far(meters) to consider
                    # paths = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),
                    #             # caDict means conflict
                    #             graph.possiblePaths(match.lanelet, possiblePathParams))
                    #possiblePathsWithInfo.extend(paths)
                    Pathset = possiblepath_calculate(matching=match,map_graph= graph,pathLengthLimit = PathLengthLimit)
                    paths2 = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),Pathset) # caDict means conflict
                    possiblePathsWithInfo.extend(paths2)
                vehicleState.pathsWithInformation = possiblePathsWithInfo
                activeObjects[track.track_id] = vehicleState
            else:
                vehicleState = activeObjects[track.track_id]
            vehicleState.update(currentMs)

        prediction_utilities.removeInactiveObjects(activeObjects, timestamp)
        # TODO: continue here - calculate matching, build lanelet->critical area dictionary, associate track -> next ca, estimate state


        if visualize:
            plt.sca(axes)
            drawing_utils.draw_vehicle_states(activeObjects, axes, patchesDict, textDict)
            prediction_utilities.cleanDrawingDicts(activeObjects, patchesDict, textDict)
            title_text.set_text("\nts = {}".format(timestamp))

            dependency_node,dependency_edges,rightofway_info,front_car_pairs,conflicting_car_pairs = dependency_calculate(activeObjects,track_dictionary,timestamp,Default_gap,
                                                                    Deviation_alongarcCoordinate,Time_gap,Time_difference_max,Distance_difference_max)
            hidden_intention = hidden_intentions_initialize(activeObjects,front_car_pairs,conflicting_car_pairs,Deviation_alongarcCoordinate,Distance_difference_max)
            plt.sca(axes2)
            G.clear()
            axes2.cla()
            G.add_nodes_from(dependency_node)
            G.add_edges_from(dependency_edges)
            nx.draw_circular(G,with_labels = True)

            end_time = time.time()
            plt.pause(max(0.001, timestamp_delta_ms / 1000. - (end_time - start_time)))
        timestamp += timestamp_delta_ms
    if visualize:
        plt.ioff()
    return


if __name__=="__main__":
    main()