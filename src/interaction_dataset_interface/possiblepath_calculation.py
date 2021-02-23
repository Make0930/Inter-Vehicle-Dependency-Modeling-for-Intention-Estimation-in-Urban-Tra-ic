import prediction_utilities
import predictiontypes
import lanelet2
import lanelet2.core
import lanelet2.geometry
import lanelet2.routing

def possiblepath_calculate(matching,map_graph,pathLengthLimit):
    Pathset = []
    PathLength = []
    #Longer_path_index = []

    if matching is None:
        return []
    stack = [matching.lanelet, [matching.lanelet], lanelet2.geometry.length2d(matching.lanelet)]
    while stack:
        length = stack.pop()
        path = stack.pop()
        node = stack.pop()
        if not map_graph.following(node):
            Pathset.append(path)
            PathLength.append(length)
        if map_graph.following(node):
            if node in path[:-1]:
                continue
            elif length > pathLengthLimit:
                Pathset.append(path[:-1])
                PathLength.append(length)
                continue
            for ll in map_graph.following(node):
                stack.append(ll)
                stack.append(path + [ll])
                stack.append(length + lanelet2.geometry.length2d(ll))

    # for i in range(len(Pathset)-1):
    #     for j in range(i + 1,len(Pathset)):
    #         if Pathset[i][-1] == Pathset[j][-1] and abs(PathLength[i] - PathLength[j]) > pathLengthDifference:
    #             if PathLength[i] > PathLength[j]:
    #                 Longer_path_index.append(i)
    #             else:
    #                 Longer_path_index.append(j)
    # if Longer_path_index:
    #     for k in Longer_path_index:  #czxc
    #         del Pathset[k - Longer_path_index.index(k)]
    return Pathset