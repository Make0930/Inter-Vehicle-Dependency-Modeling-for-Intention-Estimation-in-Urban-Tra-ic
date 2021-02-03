import lanelet2
import lanelet2.geometry
import lanelet2.io
import lanelet2.traffic_rules
import lanelet2.routing
from lanelet2.core import BasicPoint2d
import networkx as nx  # graph stuff
import predictiontypes


class BasicLaneletRelationAnalysis:
    def __init__(self, ll_id):
        self.ll_id = ll_id  # type: LaneletId

        self.conflicting = []  # type: typing.List[LaneletId]
        self.diverges_into = []  # type: typing.List[LaneletId]
        self.divergent_alternatives = []  # type: typing.List[LaneletId]
        self.converged_from = []  # type: typing.List[LaneletId]

        # TODO: include length, etc.

    def analyze(self, ll_map, routing_graph):
        self.conflicting = []
        self.diverges_into = []
        self.divergent_alternatives = []
        self.converged_from = []

        ll = ll_map.laneletLayer.get(self.ll_id)

        self.conflicting = [it.id for it in routing_graph.conflicting(ll)]

        predecessors = []
        for previous_relation in routing_graph.previousRelations(ll):
            if previous_relation.relationType == lanelet2.routing.RelationType.Successor:
                predecessors.append(previous_relation.lanelet.id)
                prev_ll = previous_relation.lanelet
                for foll in routing_graph.followingRelations(prev_ll):
                    if foll.relationType == lanelet2.routing.RelationType.Successor and foll.lanelet.id != ll.id:
                        self.divergent_alternatives.append(foll.lanelet.id)

        if len(predecessors) > 1:
            self.converged_from = predecessors    #the set of succesors

        following = []
        for foll in routing_graph.followingRelations(ll):
            if foll.relationType == lanelet2.routing.RelationType.Successor:
                following.append(foll.lanelet.id)

        if len(following) > 1:
            self.diverges_into = following

def getAllCriticalAreas(laneletmap, graph, mergeDistance):
    mergeDistanceSqr = mergeDistance * mergeDistance
    allCriticalAreas = predictiontypes.CriticalAreas()
    for lanelet in laneletmap.laneletLayer:
        analysis = BasicLaneletRelationAnalysis(lanelet.id)
        analysis.analyze(laneletmap, graph)       #get the successors(converged_from) and followers(diverges_into)
        involvedLanelets = analysis.diverges_into
        if len(involvedLanelets) > 1:           #if a lanelet has more than one following lanelets, it will diverging
            involvedLanelets.append(lanelet.id)     #add the discussed (center) lanelet
            centerLine = lanelet2.geometry.to2D(lanelet.centerline)   #the discussed (center) lanelet centerline
            criticalArea = predictiontypes.CriticalArea(involvedLanelets=involvedLanelets, map=laneletmap,
                                                        center=centerLine[-1],
                                                        description="diverging")   #center is the last point of the lanelet
            mergedIn = False
            for otherArea in allCriticalAreas:
                if distanceSqr(otherArea, criticalArea) < mergeDistanceSqr:
                    otherArea.mergeWith(criticalArea)
                    mergedIn = True
                    break
            if not mergedIn:
                allCriticalAreas.add(criticalArea)
        if len(analysis.conflicting) > 0:       #if a lanelet has a conflicting lanelet
            for conflictingLaneletId in analysis.conflicting:
                confLanelet = laneletmap.laneletLayer.get(conflictingLaneletId)
                conflictCenter = lanelet2.geometry.intersectCenterlines2d(lanelet, confLanelet)  #the common line string of 2 lanelets
                if len(conflictCenter) > 0:
                    conflictArea = predictiontypes.CriticalArea(involvedLanelets=[lanelet.id, conflictingLaneletId],
                                                                center=conflictCenter[0], description="conflicting",
                                                                map=laneletmap)
                    mergedIn = False
                    for otherArea in allCriticalAreas:
                        if distanceSqr(otherArea, conflictArea) < mergeDistanceSqr:
                            otherArea.mergeWith(conflictArea)
                            mergedIn = True
                    if not mergedIn:
                        allCriticalAreas.add(conflictArea)
        # TODO: fix so that divergings aren't always also conflicting
    return allCriticalAreas


def distanceSqr(ob1, ob2):
    diffx = ob1.x - ob2.x
    diffy = ob1.y - ob2.y
    return (diffx * diffx) + (diffy * diffy)


def createLaneletCriticalAreaDict(criticalAreas):
    returnDict = dict()
    for ca in criticalAreas :
        for lanelet in ca.involvedLanelets:
            if lanelet in returnDict:
                returnDict[lanelet].append(ca)
            else:
                returnDict[lanelet]=[ca]
    return returnDict