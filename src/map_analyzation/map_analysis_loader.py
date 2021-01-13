import argparser
import os
import predictiontypes
import yaml
import lanelet2

if __name__ == '__main__' and __package__ is None:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')


def readCriticalAreas(infile, map):
    print('loading critical area from file: ' + infile)
    criticalAreas = predictiontypes.CriticalAreas()
    with open(infile) as f:
        dict = yaml.load(f)
    criticalAreas.fromDict(dict, map)
    return criticalAreas


def main():
    args = argparser.parse_arguments(ROOT_PATH)

    print('loading map')
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args['lat_origin'], args['lon_origin']))
    laneletmap = lanelet2.io.load(args['lanelet_map'], projector)
    criticalAreas = readCriticalAreas(args['critical_areas_file'], laneletmap)
    print('critical areas read: ')
    print(yaml.safe_dump(list(map(lambda x: x.toDict(), criticalAreas.critical_areas))))


if __name__ == "__main__":
    main()
