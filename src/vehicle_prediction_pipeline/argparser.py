import argparse
import yaml
import os
import sys
import pprint

def parse_arguments(root_path) :
    parser = argparse.ArgumentParser(description='This tool is a prototype implementation of the vehicle prediction '
                                                 'approach described in the dissertation of Jannik Quehl. ')
    parser.add_argument('-lm', '--lanelet_map', help='Path to the lanelet-map', required=False)
    parser.add_argument('-c', '--config', help='path to config file', required=False)

    parser.add_argument('-d', '--debug', help='show debug output', required=False, action="store_true")
    args = vars(parser.parse_args())

    config_file = args['config']
    config = None
    if config_file and os.path.exists(config_file) :
        try :
            file_stream = open(config_file)
            config = yaml.safe_load(file_stream)
        except IOError as e:
            print('couldn\'t open config file: ')
            print(e)
            sys.exit(1)

    else :
        args['config'] = root_path + '/config/config.yaml'
        try :
            file_stream = open(args['config'])
            config = yaml.safe_load(file_stream)
        except IOError as e:
            print('couldn\'t open standard config file: ')
            print(e)
            sys.exit(1)

    if not config :
        print('No valid configuration found.')
        sys.exit(1)

    try:
        # necessary params that can be in args or config file - args have priority
        if not args['lanelet_map'] :
            args['lanelet_map'] = config['lanelet_map']
        if not args['debug'] :
            args['debug'] = config['debug']

        # params only in config
        # args['param'] = config['param']

        # optional params
        if 'predictions_out' in config:
            args['predictions_out'] = config['predictions_out']
        if 'lat_origin' in config:
            args['lat_origin'] = config['lat_origin']
        if 'lon_origin' in config:
            args['lon_origin'] = config['lon_origin']
        if 'critical_areas_file' in config:
            args['critical_areas_file'] = config['critical_areas_file']
        if 'interaction_trackfile' in config:
            args['interaction_trackfile'] = config['interaction_trackfile']
        if 'critical_area_sim_thresh' in config:
            args['critical_area_sim_thresh'] = config['critical_area_sim_thresh']
        if 'visualize' in config:
            args['visualize'] = config['visualize']

    except KeyError :
        print('missing argument in config file, exiting...')
        sys.exit(1)

    if args['debug'] :
        print('loaded following configuration: ')
        pprint.pprint(args, width=1)

    return args