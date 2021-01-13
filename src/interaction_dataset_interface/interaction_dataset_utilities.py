import dataset_types

def sortTracks(track_dictionary) :
    sortedTracks = track_dictionary
    sorted(track_dictionary, key=lambda key, value : value.time_stamp_ms_first)
    return sortedTracks
#at every time stamp, for  every car id, determine if it is showing in the map according to the first and last time stamp
def getVisibleTracks(timestamp, track_dictionary) :
    visibleObjects = []
    for key, value in track_dictionary.items() :
        assert isinstance(value, dataset_types.Track)
        if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last :
            visibleObjects.append(value)
    return visibleObjects

def getVisibleMotionStates(timestamp, track_dictionary) :
    visibleMSs = []
    for key, value in track_dictionary.items() :
        assert isinstance(value, dataset_types.Track)
        if value.time_stamp_ms_first <= timestamp <= value.time_stamp_ms_last :
            visibleMSs.append(value.motion_states[timestamp])
    return visibleMSs