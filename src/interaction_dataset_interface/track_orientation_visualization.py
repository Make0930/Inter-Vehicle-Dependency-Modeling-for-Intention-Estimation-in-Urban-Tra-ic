import lanelet2
import lanelet2.geometry
import lanelet2.core
import argparser
import os
from random import randint
import matplotlib.pyplot as plt
from matplotlib import cm
import map_analyzer
import dataset_reader
import math
import predictiontypes
import prediction_utilities

if __name__ == '__main__' and __package__ is None:
    import sys

    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')


def main():
    args = argparser.parse_arguments(ROOT_PATH)

    print('loading map')
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args['lat_origin'], args['lon_origin']))
    laneletmap = lanelet2.io.load(args['lanelet_map'], projector)
    trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                 lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(laneletmap, trafficRules)

    print('analyzing map')
    criticalAreas = map_analyzer.getAllCriticalAreas(laneletmap, graph, args['critical_area_sim_thresh'])
    laneletCaDict = map_analyzer.createLaneletCriticalAreaDict(criticalAreas)

    print('loading trackfiles')
    track_dictionary = dataset_reader.read_tracks(args['interaction_trackfile'])
    while True:
        randomTrackId = randint(1, 115)
        if randomTrackId not in track_dictionary:
            print('trackid %s not found' % (randomTrackId))
            continue
        track = track_dictionary[randomTrackId]
        timestamp = track.time_stamp_ms_first

        vehicleState = predictiontypes.Vehicle(objectId=track.track_id, motionState=track.motion_states[timestamp],
                                               width=track.width, length=track.length)
        matchings = prediction_utilities.matchMotionState(laneletmap, track.motion_states[timestamp])
        pathsWithInfo = []

        possiblePathParams = lanelet2.routing.PossiblePathsParams()
        possiblePathParams.includeShorterPaths = True
        possiblePathParams.includeLaneChanges = False

        for match in matchings:
            possiblePathParams.routingCostLimit = lanelet2.geometry.approximatedLength2d(match.lanelet) + 150
            paths = map(lambda x: predictiontypes.PathWithInformation(laneletPath=x, caDict=laneletCaDict),
                        graph.possiblePaths(match.lanelet, possiblePathParams))
            pathsWithInfo.extend(paths)
        vehicleState.pathsWithInformation = pathsWithInfo

        trackOrientations = []
        timestamps = []
        pathOrientations = [[] for j in range(len(pathsWithInfo))]

        dt = 100
        while timestamp <= track.time_stamp_ms_last:
            timestamps.append(timestamp)
            trackOrientations.append(track.motion_states[timestamp].psi_rad)
            for i in range(len(pathsWithInfo)):
                basicPoint = lanelet2.core.BasicPoint2d(track.motion_states[timestamp].x,
                                                        track.motion_states[timestamp].y)
                arcCoordinates = lanelet2.geometry.toArcCoordinates(pathsWithInfo[i].centerline, basicPoint)
                pathOrientations[i].append(pathsWithInfo[i].getOrientationAtArcLength(arcCoordinates.length))
            timestamp += dt

        fig, axes = plt.subplots(1, 1)
        fig.canvas.set_window_title("Orientation of Track %s and its matching paths" % (randomTrackId))
        plt.xlim(track.time_stamp_ms_first, track.time_stamp_ms_last)
        plt.ylim(-math.pi, math.pi)
        colors = ['g', 'b', 'c', 'm', 'y', 'k', 'orange', 'aqua', 'lime']
        markers = ['x', '+', 'v', '^', '1', '2', '3', '4', '5']
        plt.ion()
        plt.show()
        plt.sca(axes)
        plt.scatter(timestamps, trackOrientations, c='r', s=2, label='Track orientation')
        plt.plot(timestamps, trackOrientations, c='r')
        for i in range(len(pathsWithInfo)):
            plt.scatter(timestamps, pathOrientations[i], c=colors[i], label='Path number %s' % (i), marker=markers[i])
            plt.plot(timestamps, pathOrientations[i], c=colors[i])
        plt.legend()
        plt.waitforbuttonpress()
        plt.ioff()


if __name__ == "__main__":
    main()
