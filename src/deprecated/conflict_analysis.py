

def getAllConflictingAreas(laneletmap, graph):
    relationAnalyses = []
    all_critical_areas = predictiontypes.CriticalAreas()
    laneletList = []
    for lanelet in laneletmap.laneletLayer:
        laneletList.append(lanelet)
        analysis = BasicLaneletRelationAnalysis(lanelet.id)
        analysis.analyze(laneletmap, graph)
        for conflictingLaneletId in analysis.conflicting:
            conflictingLanelet = laneletmap.laneletLayer.get(conflictingLaneletId)
            center = lanelet2.geometry.intersectCenterlines2d(lanelet, conflictingLanelet)
            if len(center) > 0:
                area = predictiontypes.ConflictingArea([lanelet.id, conflictingLaneletId], center[0])
                if area not in all_critical_areas:
                    all_critical_areas.add(area)
        # TODO: merge very close critical areas
        # TODO: add different kinds of conflicts?
    print(laneletList)
    return all_critical_areas


def getAllDivergingPoints(laneletmap, graph):
    allDivergingPoints = predictiontypes.CriticalAreas()
    laneletList = []
    for lanelet in laneletmap.laneletLayer:
        laneletList.append(lanelet)
        analysis = BasicLaneletRelationAnalysis(lanelet.id)
        analysis.analyze(laneletmap, graph)
        followers = analysis.diverges_into
        if len(followers) > 1:
            centerLine = lanelet.centerline2d()
            allDivergingPoints.add(
                predictiontypes.DivergingPoint(origin=lanelet.id, followers=followers, center=centerLine[-1]))
    return allDivergingPoints
