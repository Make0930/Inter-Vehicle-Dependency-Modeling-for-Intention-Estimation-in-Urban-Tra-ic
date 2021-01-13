from lanelet2_matching.python import Pose2d
from lanelet2_matching.python import Object2d
from lanelet2_matching.python import getDeterministicMatches
import lanelet2.core

def matchMotionState(laneletmap, motionState) :
    pose = Pose2d(motionState.x, motionState.y, motionState.psi_rad)
    matchingObject = Object2d(pose=pose)
    return getDeterministicMatches(laneletmap, matchingObject, 0)

def main():
    pose = Pose2d(1,2,3)
    obj=Object2d(pose=pose)
    print(obj.pose)

if __name__ == "__main__":
    main()


def getPathCenterLine(llPath):
    centerline=[]
    for ll in llPath:
        centerline.extend(ll.centerline)
    return centerline


def removeInactiveObjects(activeObjects, timestamp):
    for id in list(activeObjects):
        if activeObjects[id].currentState.motionState.time_stamp_ms < timestamp:
            del activeObjects[id]


def cleanDrawingDicts(activeObjects, patchesDict, textDict):
    for id in list(patchesDict):
        if id not in activeObjects:
            patchesDict[id].remove()
            patchesDict.pop(id)
            textDict[id].remove()
            textDict.pop(id)