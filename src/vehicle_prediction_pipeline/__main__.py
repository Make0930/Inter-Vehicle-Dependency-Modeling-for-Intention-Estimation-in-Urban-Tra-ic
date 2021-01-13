import os
import processing_pipeline_edit
import threading # only for plots

if __name__=='__main__' and __package__ is None:
    import sys
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))
ROOT_PATH = os.path.join(os.path.dirname(__file__), '../..')


def main() :
    from argparser import parse_arguments
    args = parse_arguments(ROOT_PATH)
    print(ROOT_PATH)
    processing_pipeline_edit.startProcessingPipeline(args)


if __name__=="__main__":
    main()