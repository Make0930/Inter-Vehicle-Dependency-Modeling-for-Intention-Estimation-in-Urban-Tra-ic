import lanelet2
import argparser
import os
import map_analyzer
import yaml

if __name__ == '__main__' and __package__ is None:
    import sys

    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')


def writeOutCriticalAreas(criticalAreas, outfile):
    print('writing critical areas file to: ' + outfile)
    outData = list(map(lambda x: x.toDict(), criticalAreas.critical_areas))
    print(yaml.safe_dump(outData))
    with open(outfile, 'w') as f:
        yaml.dump(outData, f)


def main():
    args = argparser.parse_arguments(ROOT_PATH)

    print('loading map')
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args['lat_origin'], args['lon_origin']))
    laneletmap = lanelet2.io.load(args['lanelet_map'], projector)
    trafficRules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                 lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(laneletmap, trafficRules)

    print('analyzing map')
    # criticalAreas = map_analyzer.getAllConflictingAreas(laneletmap, graph)
    # divergingPoints = map_analyzer.getAllDivergingPoints(laneletmap, graph)
    # criticalAreas.extend(divergingPoints)
    criticalAreas = map_analyzer.getAllCriticalAreas(laneletmap, graph, args['critical_area_sim_thresh'])

    print('writing map analysis')
    writeOutCriticalAreas(criticalAreas, args['critical_areas_file'])


if __name__ == "__main__":
    main()
