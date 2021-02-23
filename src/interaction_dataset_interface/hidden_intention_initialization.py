import lanelet2
import lanelet2.core
import lanelet2.geometry
import lanelet2.routing
import math

def hidden_intentions_initialize(activeObjects,front_car_pair,conflicting_car_pair,Deviation_alongarcCoordinate,distance_difference_max):
    # determine the hidden intention
    hidden_intentions = {}
    #for each vehicle (key)
    for key in activeObjects.keys():
        hidden_intentions[key] = []
        diverging_area = 0
        for path_idx in range(len(activeObjects[key].pathsWithInformation)):
            # skip, if the divering-area has calculated
            if diverging_area:
                break
            # check whether the car have critical Areas in this path
            if activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates \
                    and activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[0][1].length >= 0:
                # calculate the start critical area of each path
                start_index = 0
                # skip, if the car arc_length larger than the furthest critical area arc length
                if activeObjects[key].arcCoordinatesAlongPaths[path_idx].length >= \
                        activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[-1][1].length:
                    continue
                for i in range(len(activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates)):
                    description = activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[i][
                        0].description
                    if (description == 'diverging' or description == 'unified') and \
                            activeObjects[key].arcCoordinatesAlongPaths[path_idx].length <= \
                            activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[i][1].length + \
                            activeObjects[
                                key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[i][0].radius:
                        path_index = path_idx
                        diverging_area = activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[i]
                        distance = activeObjects[key].pathsWithInformation[path_idx].criticalAreasWithCoordinates[i][
                                       1].length - activeObjects[key].arcCoordinatesAlongPaths[path_idx].length
                        break
        # if the diverging area is close to the car,we need to classify the paths
        if diverging_area and distance < distance_difference_max:
            Lanlelet_in_path = {}
            Lanelet_orientation = {}
            Paths_in_same_orientation = {}
            Diverging_lanelets_psi = []
            diverging_area_length = diverging_area[1].length
            for path_idx in range(len(activeObjects[key].pathsWithInformation)):
                laneletssequence = activeObjects[key].pathsWithInformation[path_idx].laneletSequence.lanelets()
                for ll in laneletssequence:
                    ll_point = lanelet2.core.BasicPoint2d(ll.centerline[-1].x, ll.centerline[-1].y)
                    ll_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                        activeObjects[key].pathsWithInformation[path_idx].centerline,
                        ll_point)
                    if ll_arcCoordinate.length == diverging_area_length and ll_arcCoordinate.distance < Deviation_alongarcCoordinate:
                        LL_next = laneletssequence[laneletssequence.index(ll) + 1]
                        Lanlelet_in_path[path_idx] = LL_next
                        repetition = 0
                        for LL in Diverging_lanelets_psi:
                            if LL[0] == LL_next:
                                repetition = 1
                                break
                        if repetition:
                            break
                        LL_next_psi = math.atan2(LL_next.centerline[-1].y - LL_next.centerline[-2].y,
                                                 LL_next.centerline[-1].x - LL_next.centerline[-2].x)
                        Diverging_lanelets_psi.append((LL_next, LL_next_psi))
                        break
            # the intention is left and right
            if len(Diverging_lanelets_psi) == 2:
                if Diverging_lanelets_psi[0][1] * Diverging_lanelets_psi[1][1] > 0:
                    if Diverging_lanelets_psi[0][1] > Diverging_lanelets_psi[1][1]:
                        Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'left'
                        Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'right'
                    elif Diverging_lanelets_psi[0][1] < Diverging_lanelets_psi[1][1]:
                        Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'right'
                        Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'left'
                if Diverging_lanelets_psi[0][1] * Diverging_lanelets_psi[1][1] < 0:
                    if abs(Diverging_lanelets_psi[0][1]) + abs(Diverging_lanelets_psi[1][1]) > math.pi:
                        if Diverging_lanelets_psi[0][1] > 0:
                            Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'right'
                            Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'left'
                        else:
                            Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'left'
                            Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'right'
                    else:
                        if Diverging_lanelets_psi[0][1] > 0:
                            Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'left'
                            Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'right'
                        else:
                            Lanelet_orientation[Diverging_lanelets_psi[0][0]] = 'right'
                            Lanelet_orientation[Diverging_lanelets_psi[1][0]] = 'left'

            # the intention is left , go straight and right
            # if len(Path_diverging_lanelets) == 3:
            for ll in Lanelet_orientation.keys():
                Paths_in_same_orientation[Lanelet_orientation[ll]] = []
                for path in Lanlelet_in_path.keys():
                    if Lanlelet_in_path[path] == ll:
                        Paths_in_same_orientation[Lanelet_orientation[ll]].append(path)

            for orientation in Paths_in_same_orientation.keys():
                front_cars = 0
                conflicting_cars = 0
                for path in Paths_in_same_orientation[orientation]:
                    if front_car_pair and front_car_pair[key]:
                        for pa1 in front_car_pair[key].keys():
                            if pa1 == path and front_car_pair[key][pa1]:
                                front_cars = 1
                                # possible_intent = 'after ' + str(i[0])
                                if orientation + ' after ' + str(front_car_pair[key][pa1][0]) not in hidden_intentions[key]:
                                    hidden_intentions[key].append(
                                        orientation + ' after ' + str(front_car_pair[key][pa1][0]))
                    if conflicting_car_pair and conflicting_car_pair[key]:
                        for pa2 in conflicting_car_pair[key].keys():
                            if pa2 == path and conflicting_car_pair[key][pa2]:
                                conflicting_cars = 1
                                for k in conflicting_car_pair[key][pa2]:
                                    if orientation + ' after ' + str(k) not in hidden_intentions[key]:
                                        hidden_intentions[key].append(orientation + ' after ' + str(k))
                # all paths in this orientation don't have front or conflicting cars
                if not front_cars and not conflicting_cars:
                    if orientation not in hidden_intentions[key]:
                        hidden_intentions[key].append(orientation)
                if not front_cars and conflicting_cars:
                    if orientation + ' first' not in hidden_intentions[key]:
                        hidden_intentions[key].append(orientation + ' first')

        # if all the path don't have divering area or the diverging area is too far.
        if not diverging_area or (diverging_area and distance >= distance_difference_max):
            front_cars = 0
            conflicting_cars = 0
            if front_car_pair and front_car_pair[key]:
                for i in front_car_pair[key].values():
                    if i:
                        front_cars = 1
                        # possible_intent = 'after ' + str(i[0])
                        if 'go after ' + str(i[0]) not in hidden_intentions[key]:
                            hidden_intentions[key].append('go after ' + str(i[0]))
            if conflicting_car_pair and conflicting_car_pair[key]:
                for j in conflicting_car_pair[key].values():
                    if j:
                        conflicting_cars = 1
                        for k in j:
                            if 'go after ' + str(k) not in hidden_intentions[key]:
                                hidden_intentions[key].append('go after ' + str(k))
            if not front_cars and not conflicting_cars:
                hidden_intentions[key].append('go along the Path')
            if not front_cars and conflicting_cars:
                hidden_intentions[key].append('go first')

        # if the car is stopping without front cars or conflicting cars
        if (activeObjects[key].currentVelocity < 0.3 or activeObjects[key].acceleration < -4) and (
                not front_car_pair[key] or not front_car_pair[key][0]) and (
                not conflicting_car_pair[key] or not conflicting_car_pair[key][0]):
            hidden_intentions[key].append('stop')

    return hidden_intentions