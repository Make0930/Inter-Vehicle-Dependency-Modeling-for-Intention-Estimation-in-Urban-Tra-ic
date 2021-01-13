import lanelet2
import argparser
import os
import drawing_utils
import matplotlib.pyplot as plt

if __name__=='__main__' and __package__ is None:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')

def main() :
    args = argparser.parse_arguments(ROOT_PATH)

    print('loading map')
    projector = lanelet2.projection.UtmProjector(lanelet2.io.Origin(args['lat_origin'], args['lon_origin']))
    laneletmap = lanelet2.io.load(args['lanelet_map'], projector)

    print('drawing map')
    fig, axes = plt.subplots(1, 1)
    fig.canvas.set_window_title("Map Visualization")
    drawing_utils.draw_lanelet_map(laneletmap, axes)

    plt.show()

if __name__=="__main__":
    main()