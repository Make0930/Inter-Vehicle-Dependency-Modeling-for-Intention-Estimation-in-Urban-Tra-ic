# import copy
import lanelet2.core
import lanelet2.geometry
import math
import numpy as np
from lanelet2.routing import LaneletPath
from lanelet2.core import ConstLanelet,ConstLineString2d
import dataset_types
import processing_pipeline

def short_id(id64):
    """Given a 64-bit id, make a shorter 16-bit one.
       https://github.com/nedbat/coveragepy/blob/master/coverage/debug.py#L173
    """
    id16 = 0
    for offset in range(0, 64, 16):
        id16 ^= id64 >> offset
    return id16 & 0xFFFF


class CriticalArea:

    # def __init__(self, center, involvedLanelets=[], originLanelets=[], followingLanelets=[], conflictinglanelets=[], radius=3.0,
    #              description=""):
    def __init__(self, center, map, involvedLanelets=[], radius=3.0, description=""):
        self.involvedLanelets = involvedLanelets
        self.involvedLanelets.sort()
        self.arcCoordinates = dict()
        for lanelet in self.involvedLanelets:
            centerline = lanelet2.geometry.to2D(map.laneletLayer.get(lanelet).centerline)
            # centerline = map.laneletLayer.get(lanelet).centerline
            basicCenter = lanelet2.core.BasicPoint2d(center.x, center.y)
            self.arcCoordinates[lanelet] = lanelet2.geometry.toArcCoordinates(centerline, basicCenter)

        # self.conflictinglanelets = conflictinglanelets  # type: typing.List[LaneletId]
        # self.conflictinglanelets.sort()
        # self.originLanelets = originLanelets  # type: typing.List[LaneletId]
        # self.followingLanelets = followingLanelets  # type: typing.List[LaneletId]
        self.x = center.x  # type: float
        self.y = center.y  # type: float
        self.radius = radius  # type: float
        self.radiusSqr = radius * radius
        self.description = description  # type: str
        self.numberOfMerges = 0;

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (self.involvedLanelets == other.involvedLanelets) and (self.description == other.description)
        else:
            return False

    def toDict(self):
        return {'involvedLanelets': self.involvedLanelets,
                'x': self.x,
                'y': self.y,
                'radius': self.radius,
                'description': self.description}

    def mergeWith(self, otherArea):
        assert isinstance(otherArea, self.__class__)
        self.numberOfMerges += 1
        mergedX = (self.x * self.numberOfMerges) / (self.numberOfMerges + 1) + (otherArea.x / (self.numberOfMerges + 1))
        mergedy = (self.y * self.numberOfMerges) / (self.numberOfMerges + 1) + (otherArea.y / (self.numberOfMerges + 1))
        dx = abs(mergedX - self.x)
        dy = abs(mergedy - self.y)
        self.radius += math.sqrt((dx * dx) + (dy * dy))         # the radius is larger than before
        self.radiusSqr = self.radius * self.radius
        for otherLanelet in otherArea.involvedLanelets:
            isInThis = False
            for lanelet in self.involvedLanelets:
                if otherLanelet == lanelet:
                    isInThis = True
                    break
            if not isInThis:
                self.involvedLanelets.append(otherLanelet)
        self.involvedLanelets.sort()
        if self.description != otherArea.description:     # the area is diverging and conflicting
            self.description = "unified"


class DivergingPoint(CriticalArea):

    def __init__(self, origin, followers, center):
        origins = {origin}
        CriticalArea.__init__(self, center=center, originLanelets=origins, followingLanelets=followers,
                              description="diverging")


class ConflictingArea(CriticalArea):
    """ Area around point restricted to vehicles on specific lanelets """

    def __init__(self, lanelets, center, radius=3.0):
        CriticalArea.__init__(self, center=center, conflictinglanelets=lanelets, radius=radius,
                              description="conflicting")

    def is_object_inside(self, laneletId, x, y):
        # type: (LaneletId, float, float) -> bool

        if laneletId not in self.lanelets:
            return False

        dx = x - self.x
        dy = y - self.y
        distSqr = dx * dx + dy * dy  # could optimize this

        return distSqr <= self.radius

    def __eq__(self, other):
        if isinstance(other, self.__class__):
            return (self.conflictinglanelets == other.conflictinglanelets) and (self.description == other.description)
        else:
            return False

    def __repr__(self):
        description_str = ""
        if self.description:
            description_str = " desc=\"{}\"".format(self.description)

        return "[CriticalArea {} (x/y)=({:.1f}/{:.1f}) radius={:.1f} lanelets={}{}]".format(
            short_id(id(self)), self.x, self.y, self.radius, self.lanelets, description_str
        )


class CriticalAreas:
    def __init__(self):
        self.critical_areas = []  # type: typing.List[ConflictingArea]

    def __iter__(self):
        return iter(self.critical_areas)

    def __repr__(self):
        return "[CriticalAreas: {} areas]".format(len(self.critical_areas))

    def extend(self, otherAreas):
        assert isinstance(otherAreas, CriticalAreas)
        self.critical_areas.extend(otherAreas.critical_areas)

    def add(self, crit_area):
        self.critical_areas.append(crit_area)

    def fromDict(self, dict, map):
        for d in dict:
            involvedLaneletIds = []
            for llid in d['involvedLanelets']:
                involvedLaneletIds.append(int(llid))
            point = lanelet2.core.BasicPoint2d()
            point.x = d['x']
            point.y = d['y']
            self.add(CriticalArea(center=point, radius=d['radius'], involvedLanelets=involvedLaneletIds,
                                  description=d['description'], map=map))


class PathWithInformation:
    def __init__(self, laneletPath, caDict):   #lanelet path is a path, consist of many lanelets.  the length is lanelets number
        self.laneletSequence = lanelet2.core.LaneletSequence(laneletPath)   # cut lanelet into smallest line pieces
        self.centerline = lanelet2.geometry.to2D(self.laneletSequence.centerline)   #cut all the laneltes in this path to smallest line pieces,  made of many points
        self.criticalAreasWithCoordinates = []
        self.lengthAccumulations = []
        for ll in laneletPath:  # for each lanelet in the path
            if ll.id in caDict:      #if the lanelet is in conflicting area
                for ca in caDict[ll.id]:    #caDict is a dictionary for all conflict areas in the map, key is lanelet id,
                    basicCaCenter = lanelet2.core.BasicPoint2d(ca.x, ca.y)    # center of conflict area
                    coordinate = lanelet2.geometry.toArcCoordinates(self.centerline, basicCaCenter)
                    self.criticalAreasWithCoordinates.append((ca, coordinate))     # (two elements, first is critialarea name, second is the arccoordinate of the area along the path
        self.criticalAreasWithCoordinates.sort(key=lambda x: x[1].length)         # sort the area according to the arc -length from  the path start point
        centerlinecopy = lanelet2.core.LineString2d(0,[])   # a series of linestring, becomes longer
        for p in self.centerline:
            point = lanelet2.core.Point2d(0,p.x,p.y)
            centerlinecopy.append(point)
            self.lengthAccumulations.append(lanelet2.geometry.length(centerlinecopy))

    def getOrientationAtArcLength(self, arcLength):
        assert (arcLength >= 0)
        nextIndex = len(self.lengthAccumulations) - 1
        #print(len(self.centerline))
        #print(len(self.lengthAccumulations))
        for i in range(len(self.lengthAccumulations)):
            if self.lengthAccumulations[i] > arcLength:
                nextIndex = max(1, i)
                break
        return math.atan2(self.centerline[nextIndex].y - self.centerline[nextIndex - 1].y,
                          self.centerline[nextIndex].x - self.centerline[nextIndex - 1].x)



class MotionState:
    def __init__(self, time_stamp_ms):
        assert isinstance(time_stamp_ms, int)
        self.time_stamp_ms = time_stamp_ms
        self.x = None
        self.y = None
        self.vx = None
        self.vy = None
        self.psi_rad = None

    def __str__(self):
        return "MotionState: " + str(self.__dict__)


class VehicleState:

    def __init__(self, motionState):
        self.motionState = motionState


class Vehicle:

    def updateArcCoordinates(self):
        self.arcCoordinatesAlongPaths = []
        for pathWithInfo in self.pathsWithInformation:  #len(pathWithInfo.centerline) = 174   for each path
            basicMsCenter = lanelet2.core.BasicPoint2d(self.currentState.motionState.x, self.currentState.motionState.y)  #x,y center of car
            # print(lanelet2.geometry.toArcCoordinates(pathWithInfo.centerline, basicMsCenter).distance) # the distance between car and path
            # print(lanelet2.geometry.toArcCoordinates(pathWithInfo.centerline, basicMsCenter).length)   # the car move length along the path
            self.arcCoordinatesAlongPaths.append(
                lanelet2.geometry.toArcCoordinates(pathWithInfo.centerline, basicMsCenter))

            #self.laneletMatchings.append(processing_pipeline.startProcessingPipeline.match)
    def updatepathOrientation(self):
        self.pathOrientation = [[0, 0] for i in range(len(self.pathsWithInformation))]
        for j in range(len(self.pathsWithInformation)):    # for each path     j = path id
            arcLength = self.arcCoordinatesAlongPaths[j].length
            assert (arcLength >= 0)
            nextIndex = len(self.pathsWithInformation[j].lengthAccumulations) - 1
            for i in range(len(self.pathsWithInformation[j].lengthAccumulations)):
                if self.pathsWithInformation[j].lengthAccumulations[i] > arcLength:
                    nextIndex = max(1, i)
                    self.pathOrientation[j][0] = nextIndex
                    break
            self.pathOrientation[j][1] =math.atan2(self.pathsWithInformation[j].centerline[nextIndex].y - self.pathsWithInformation[j].centerline[nextIndex - 1].y,
                              self.pathsWithInformation[j].centerline[nextIndex].x - self.pathsWithInformation[j].centerline[nextIndex - 1].x)


    def updatepossiblepaths(self):
         delete_index = []
         for arcCoordinate_idx in range(len(self.arcCoordinatesAlongPaths)):  #check each path
             if abs(self.arcCoordinatesAlongPaths[arcCoordinate_idx].distance) > 4:
                 delete_index.append(arcCoordinate_idx)
         for orientation_idx in range(len(self.pathOrientation)):
             if (self.currentState.motionState.psi_rad *self.pathOrientation[orientation_idx][1]) < 0 \
                     and (abs(self.pathOrientation[orientation_idx][1]) + abs(self.currentState.motionState.psi_rad)) > math.pi:
                 if self.pathOrientation[orientation_idx][1] > 0:
                     arc_difference = 2 * math.pi - self.pathOrientation[orientation_idx][1] + self.currentState.motionState.psi_rad
                 else:
                     arc_difference = 2 * math.pi + self.pathOrientation[orientation_idx][1] - self.currentState.motionState.psi_rad
             else:
                 arc_difference = abs(self.pathOrientation[orientation_idx][1] - self.currentState.motionState.psi_rad)
             if arc_difference > math.pi / 2 and orientation_idx not in delete_index:
                 delete_index.append(orientation_idx)
         if delete_index:
             for i in delete_index:
                 del self.arcCoordinatesAlongPaths[i-delete_index.index(i)]
                 del self.pathsWithInformation[i-delete_index.index(i)]
                 del self.pathOrientation[i-delete_index.index(i)]



    # for path in self.pathsWithInformation:  #check each path
        #     i = 0
        #     for ll_index in range(len(self.laneletMatchings)):    #for each current lanelet id
        #         if (self.laneletMatchings[ll_index].lanelet in path.laneletSequence):
        #             i = 1
        #             break
        #     if i == 0:
        #         del self.arcCoordinatesAlongPaths[self.pathsWithInformation.index(path)]
        #         del self.centerlineIndexAlongPaths[self.pathsWithInformation.index(path)]
        #         del self.pathsWithInformation[self.pathsWithInformation.index(path)]
    def updatepreviousVelocity(self):
        self.previousVelocity = self.currentVelocity
        if self.timestamp_first == self.currentState.motionState.time_stamp_ms:
            self.previousVelocity = math.sqrt(self.currentState.motionState.vx ** 2 +
                                             self.currentState.motionState.vy ** 2)

    def updatecurrentVelocity(self):
        self.currentVelocity = math.sqrt(self.currentState.motionState.vx ** 2 +
                                    self.currentState.motionState.vy ** 2)

    def updateacceleration(self):
        #if self.timestamp_first != self.currentState.motionState.time_stamp_ms:
        self.acceleration = (self.currentVelocity - self.previousVelocity) / 0.1







    def __init__(self, objectId=-1, motionState=MotionState(-1),
                 pathsWithInformation=[],
                 laneletMatchings=[], arcCoordinatesAlongPaths = [],width=2, length=4,pathOrientation = [],previousVelocity = 0, currentVelocity = 0,acceleration = 0, timestamp_first =0):
        self.width = width
        self.length = length
        self.objectId = objectId
        self.timestamp_first = timestamp_first
        self.currentState = VehicleState(motionState)
        self.pathsWithInformation = pathsWithInformation
        self.arcCoordinatesAlongPaths = arcCoordinatesAlongPaths
        self.updateArcCoordinates()
        self.laneletMatchings = laneletMatchings
        self.color = np.random.rand(3)
        self.pathOrientation = pathOrientation
        self.previousVelocity = previousVelocity
        self.currentVelocity = currentVelocity
        self.acceleration = acceleration
        self.updatepathOrientation()
        self.updatepossiblepaths()
        self.updatepreviousVelocity()
        self.updatecurrentVelocity()
        self.updateacceleration()

    def update(self, motionsState):
        self.updatepreviousVelocity()
        self.currentState = VehicleState(motionsState)
        self.updateArcCoordinates()
        self.updatepathOrientation()
        self.updatepossiblepaths()
        self.updatecurrentVelocity()
        self.updateacceleration()


class HomotopyClass:
    """ Identifying information about a class of trajectories that are similar as defined by
        the order objects enter certain critical areas
    """

    def __init__(self, critical_areas):
        self.critical_areas = critical_areas  # type: CriticalAreas
        self.objects_entered_area_order = [[] for _ in critical_areas]  # type: typing.List[typing.List[ObjectId]]
        self.blacklist = [[] for _ in critical_areas]  # type: typing.List[typing.List[ObjectId]]

        # instance variables for later checking the actual sequence of objects entering critical areas
        self.check_objects_entered_area_order = [[] for _ in critical_areas]  # type: typing.List[typing.List[ObjectId]]
        self.checks_till_now_successful = True

    def add_obj_pos(self,
                    object_id,  # type: ObjectId
                    lanelet_id,  # type: LaneletId
                    x,  # type: float
                    y,  # type: float
                    blacklist_object=False  # type: bool
                    ):
        """ Add current position of object (only affects self if entering critical area for the first time)
            This is used for generating the homotopy class sequentially.
            Objects can be blacklisted by specifying blacklist_object=True (blacklisting is without regard for order).

            :return Whether at least one critical area was touched
        """
        if any(self.check_objects_entered_area_order):
            raise RuntimeError("Cannot add new object position to HomotopyClass after starting to check it.")

        touched_any_critical_area = False
        for ca, ca_order, ca_blacklist in zip(self.critical_areas, self.objects_entered_area_order, self.blacklist):
            if blacklist_object:
                # blacklisting
                if object_id not in ca_blacklist:
                    if ca.is_object_inside(lanelet_id, x, y):
                        touched_any_critical_area = True
                        ca_blacklist.append(object_id)
            else:
                # normal case - not blacklisting
                if object_id not in ca_order:
                    # first seen here -> add if within area
                    if ca.is_object_inside(lanelet_id, x, y):
                        touched_any_critical_area = True
                        ca_order.append(object_id)

        return touched_any_critical_area

    def check_and_add_obj_pos(self,
                              object_id,  # type: ObjectId
                              lanelet_id,  # type: LaneletId
                              x,  # type: float
                              y,  # type: float
                              ):
        """ Check whether current position of object (and all until now) aligns with HomotopyClass.
            :return True if all objects so far are acceptable, False if there is a htclass mismatch
        """
        if not self.checks_till_now_successful:
            return False

        for ca, ca_order_actual, ca_order_target, ca_blacklist in zip(self.critical_areas,
                                                                      self.check_objects_entered_area_order,
                                                                      self.objects_entered_area_order, self.blacklist):
            if object_id not in ca_order_actual:
                # first seen here -> add if within area
                if ca.is_object_inside(lanelet_id, x, y):
                    ca_order_actual.append(object_id)
                    # check whether this observation is valid w.r.t. to target order
                    if len(ca_order_actual) <= len(ca_order_target):
                        # must check whether last element matches
                        idx = len(ca_order_actual) - 1

                        # normal order check
                        if ca_order_actual[idx] != ca_order_target[idx]:
                            self.checks_till_now_successful = False
                            return False

                    # check blacklist
                    if object_id in ca_blacklist:
                        self.checks_till_now_successful = False
                        return False

        return True

    def still_plausible(self):
        return self.checks_till_now_successful

    def __repr__(self):
        s = "[HomotopyClass: {} critical areas\n".format(len(self.critical_areas.critical_areas))
        for ca, ca_order, ca_blacklist in zip(self.critical_areas, self.objects_entered_area_order, self.blacklist):
            if ca_order or ca_blacklist:
                s += "  " + str(ca) + "\n    Order: {}, Blacklist: {}\n".format(ca_order, ca_blacklist)
        s += "]"
        return s

    def prune(self):
        """ Remove unused critical areas (i.e. without defined order or blacklist)
            TODO: possible without deepcopy? 'tis expensive
        """
        self.critical_areas = copy.deepcopy(self.critical_areas)  # copy to avoid unwanted interactions

        indices_to_delete = []
        for idx, (ca_order, ca_blacklist) in enumerate(zip(self.objects_entered_area_order, self.blacklist)):
            if not ca_order and not ca_blacklist:
                indices_to_delete.append(idx)

        # actually delete (backwards)
        for idx in sorted(indices_to_delete, reverse=True):
            del self.critical_areas.critical_areas[idx]
            del self.objects_entered_area_order[idx]
            del self.blacklist[idx]
