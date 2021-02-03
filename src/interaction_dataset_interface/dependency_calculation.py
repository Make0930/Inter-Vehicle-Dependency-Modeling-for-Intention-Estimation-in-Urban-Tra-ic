import lanelet2
import lanelet2.core
import lanelet2.geometry
import lanelet2.routing
import prediction_utilities
import predictiontypes
from lanelet2.core import LaneletSequence
from lanelet2_matching import python
import math
import networkx as nx
from interval import Interval

def dependency_calculate(activeObjects,track_dictionary,timestamp,default_gap,Deviation_alongarcCoordinate,Time_gap,Time_difference_max,Distance_difference_max):

    dependency_node = []
    dependency_edges = []

    for i in activeObjects.keys():
        dependency_node.append(i)
    if len(activeObjects) > 1:

        # first level: Calculate vehicle dependencies with clear priorities(in a sequence) and save them in a calculated_pair
        calculated_pair = {}
        for key in activeObjects.keys():  # for each center car
            for k in range(len(activeObjects[key].pathsWithInformation)):  # for each path
                compare_distance = {}
                self_arcCoordinates = activeObjects[key].arcCoordinatesAlongPaths[k]
                for j in activeObjects.keys():  # for each other car in this path
                    if j == key or (j in calculated_pair and key in calculated_pair[
                        j]) or ((j,key) in dependency_edges and (key,j) in dependency_edges):  # skip, if center car ==  other car or center car and other car are already paird
                        continue
                    the_car_point = lanelet2.core.BasicPoint2d(track_dictionary[j].motion_states[timestamp].x,
                                                               track_dictionary[j].motion_states[timestamp].y)
                    car_arcCoordinates = lanelet2.geometry.toArcCoordinates(
                        activeObjects[key].pathsWithInformation[k].centerline, the_car_point)
                    gap = car_arcCoordinates.length - self_arcCoordinates.length
                    individual_gap = activeObjects[key].currentVelocity * Time_gap
                    gap_interval = Interval(0, max(default_gap,individual_gap), closed=False)
                    if abs(car_arcCoordinates.distance) < Deviation_alongarcCoordinate and gap in gap_interval:
                        compare_distance[j] = gap
                        if key not in calculated_pair or j not in calculated_pair[key]:
                            calculated_pair.setdefault(key, []).append(j)  # key is center car id,  value is the other cars list, which in the possible path of center car
                if compare_distance:  # if the dict not empty
                    car_index = min(compare_distance, key=compare_distance.get)
                    if (key, car_index) not in dependency_edges:
                        dependency_edges.append((key, car_index))  # key depends on j, key points to j

        # second level: Calculate the dependencies between vehicles that may conflict in the future
        paired = {}
        for key in activeObjects.keys():  # for each car A id = key
            for Pa in range(len(activeObjects[key].pathsWithInformation)):  # for each possible path of car A  pathsWithInformation[Pa]
                car_list = []
                for j in activeObjects.keys():  # for each car B id = j
                    #skip calculated cars
                    if j == key or (j in paired and key in paired[j]) or (
                            j in calculated_pair and key in calculated_pair[j]) \
                            or (key in calculated_pair and j in calculated_pair[key]):
                        continue
                    # if key ==2 and j == 7:
                    #     print()
                    dependency_key_j = 0
                    car_list.append(j)
                    for Pb in range(len(activeObjects[j].pathsWithInformation)):  # for each path of car B
                        # check whether the cars have critical Areas
                        if dependency_key_j == 0 and activeObjects[key].pathsWithInformation[
                            Pa].criticalAreasWithCoordinates and activeObjects[j].pathsWithInformation[
                            Pb].criticalAreasWithCoordinates \
                                and activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[0][
                            1].length > 0 and activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[0][
                            1].length > 0:
                            # calculate the start critical area of each path
                            start_index_A = 0
                            start_index_B = 0
                            # skip, if the car arc_length larger than the furthest critical area arc length
                            if activeObjects[key].arcCoordinatesAlongPaths[Pa].length >= \
                                activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[-1][1].length or \
                                    activeObjects[j].arcCoordinatesAlongPaths[Pb].length >= \
                                    activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[-1][1].length:
                                continue
                            for i in range(
                                    len(activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates)):
                                if activeObjects[key].arcCoordinatesAlongPaths[Pa].length < \
                                        activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[i][
                                            1].length:
                                    start_index_A = i  # critical_area_start_index for A
                                    break
                            for k in range(
                                    len(activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates)):
                                if activeObjects[j].arcCoordinatesAlongPaths[Pb].length < \
                                        activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[k][
                                            1].length:
                                    start_index_B = k  # critical_area_start_index for B
                                    break

                            # the first common critical area must be A or B
                            distance_a = 0              #distance between car a and first common critical area
                            distance_b = 0              #distance between car b and first common critical area
                            length_area_a = 0           #the first common area arc_length along the car a's path
                            length_area_b = 0           #the first common area arc_length along the car b's path
                            first_common_area = 0       #we assume the common area not exists

                            # judge if the A area in b car's path
                            # if it is i = start index for A
                            common_a = 0
                            for n in range(start_index_B,
                                           len(activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates)):
                                if \
                                activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[start_index_A][
                                    0] == \
                                        activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[n][0]:
                                    common_a = 1
                                    first_common_area_i = \
                                    activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[
                                        start_index_A][0]
                                    length_area_ai = \
                                    activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[
                                        start_index_A][1].length
                                    length_area_bi = \
                                    activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[n][1].length
                                    distance_a_i = length_area_ai - activeObjects[key].arcCoordinatesAlongPaths[
                                        Pa].length
                                    distance_b_i = length_area_bi - activeObjects[j].arcCoordinatesAlongPaths[Pb].length
                                    break

                            # judge if the B area in a car's path
                            # if it is k = start index for B
                            common_b = 0
                            for m in range(start_index_A, len(
                                    activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates)):
                                if activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[m][0] == \
                                        activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[
                                            start_index_B][0]:
                                    common_b = 1
                                    first_common_area_k = \
                                    activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[m][0]
                                    length_area_ak = \
                                    activeObjects[key].pathsWithInformation[Pa].criticalAreasWithCoordinates[m][
                                        1].length
                                    length_area_bk = \
                                    activeObjects[j].pathsWithInformation[Pb].criticalAreasWithCoordinates[
                                        start_index_B][1].length
                                    distance_b_k = length_area_bk - activeObjects[j].arcCoordinatesAlongPaths[Pb].length
                                    distance_a_k = length_area_ak - activeObjects[key].arcCoordinatesAlongPaths[
                                        Pa].length
                                    break
                            # if A and B are both common areas
                            if common_a != 0 and common_b != 0:
                                # if the difference of 2 cars'distance to the A area smaller than to B area
                                if abs(distance_a_i - distance_b_i) < abs(distance_a_k - distance_b_k):
                                    first_common_area = first_common_area_i
                                    distance_a = distance_a_i
                                    distance_b = distance_b_i
                                    length_area_a = length_area_ai
                                    length_area_b = length_area_bi
                                else:
                                    first_common_area = first_common_area_k
                                    distance_a = distance_a_k
                                    distance_b = distance_b_k
                                    length_area_a = length_area_ak
                                    length_area_b = length_area_bk
                            # if only A is common area
                            elif common_a != 0 and common_b == 0:
                                first_common_area = first_common_area_i
                                distance_a = distance_a_i
                                distance_b = distance_b_i
                                length_area_a = length_area_ai
                                length_area_b = length_area_bi

                            # if only B is common area
                            elif common_a == 0 and common_b != 0:
                                first_common_area = first_common_area_k
                                distance_a = distance_a_k
                                distance_b = distance_b_k
                                length_area_a = length_area_ak
                                length_area_b = length_area_bk
                            # otherwise the don't have common areas, check another path
                            else:
                                continue

                            #if first common area exists for car a and car b
                            if first_common_area:
            # 2.1 Determine dependencies based on vehicle speed and acceleration
                                if abs(distance_a - distance_b) <= default_gap:
                                    # if both car have front car
                                    if key in calculated_pair and j in calculated_pair:
                                        front_car_A = 0
                                        for i in calculated_pair[key]:  # i = front car id
                                            car_point_A = lanelet2.core.BasicPoint2d(
                                                track_dictionary[i].motion_states[timestamp].x,
                                                track_dictionary[i].motion_states[timestamp].y)
                                            car_arc_A = lanelet2.geometry.toArcCoordinates(
                                                activeObjects[key].pathsWithInformation[Pa].centerline, car_point_A)
                                            if abs(
                                                    car_arc_A.distance) < Deviation_alongarcCoordinate and car_arc_A.length in Interval(
                                                    activeObjects[key].arcCoordinatesAlongPaths[Pa].length, length_area_a,
                                                    closed=False):
                                                front_car_A = 1
                                                break
                                        # if no car between car a and critical area
                                        if front_car_A == 0:
                                            front_car_B = 0
                                            for k in calculated_pair[j]:
                                                car_point_B = lanelet2.core.BasicPoint2d(
                                                    track_dictionary[k].motion_states[timestamp].x,
                                                    track_dictionary[k].motion_states[timestamp].y)
                                                car_arc_B = lanelet2.geometry.toArcCoordinates(
                                                    activeObjects[j].pathsWithInformation[Pb].centerline, car_point_B)
                                                if abs(
                                                        car_arc_B.distance) < Deviation_alongarcCoordinate and car_arc_B.length in Interval(
                                                        activeObjects[j].arcCoordinatesAlongPaths[Pb].length, length_area_b,
                                                        closed=False):
                                                    front_car_B = 1
                                                    break
                                            # no car between car b and critical area
                                            if front_car_B == 0:
                                                v_a = activeObjects[key].currentVelocity
                                                v_b = activeObjects[j].currentVelocity
                                                acc_a = activeObjects[key].acceleration
                                                acc_b = activeObjects[j].acceleration
                                                time_a = 0
                                                time_b = 0
                                                if acc_a != 0 and v_a ** 2 + 2 * acc_a * distance_a >= 0:
                                                    time_a = (-v_a + math.sqrt(v_a ** 2 + 2 * acc_a * distance_a)) / acc_a
                                                else:  # acc_a == 0
                                                    if v_a > 0:
                                                        time_a = distance_a / v_a

                                                if acc_b != 0 and v_b ** 2 + 2 * acc_b * distance_b >= 0:
                                                    time_b = (-v_b + math.sqrt(v_b ** 2 + 2 * acc_b * distance_b)) / acc_b
                                                else:  # acc_b == 0
                                                    if v_b > 0:
                                                        time_b = distance_b / v_b
                                                if time_a and time_b:
                                                    # if car A and car B in the first timestamp,can't calculate acceleration
                                                    # if timestamp == track_dictionary[key].time_stamp_ms_first == track_dictionary[j].time_stamp_ms_first:
                                                    if time_a - time_b > Time_difference_max:  # b j is much faster
                                                        if (key, j) not in dependency_edges:
                                                            dependency_edges.append((key, j))
                                                    elif time_b - time_a > Time_difference_max:  # a is much faster
                                                        if (j, key) not in dependency_edges:
                                                            dependency_edges.append((j, key))
                                                    elif (abs(time_a - time_b) < Time_difference_max):
                                                        dependency_key_j = 1
                                                        if (key, j) not in dependency_edges:
                                                            dependency_edges.append((key, j))
                                                            dependency_edges.append((j, key))
                                    # if only car A has front car
                                    if key in calculated_pair and j not in calculated_pair:
                                        front_car_A = 0
                                        for i in calculated_pair[key]:  # i = front car id
                                            car_point_A = lanelet2.core.BasicPoint2d(
                                                track_dictionary[i].motion_states[timestamp].x,
                                                track_dictionary[i].motion_states[timestamp].y)
                                            car_arc_A = lanelet2.geometry.toArcCoordinates(
                                                activeObjects[key].pathsWithInformation[Pa].centerline, car_point_A)
                                            if abs(
                                                    car_arc_A.distance) < Deviation_alongarcCoordinate and car_arc_A.length in Interval(
                                                    activeObjects[key].arcCoordinatesAlongPaths[Pa].length, length_area_a,
                                                    closed=False):
                                                front_car_A = 1
                                                break
                                        if front_car_A == 0:  # no car between car A and critical area
                                            v_a = activeObjects[key].currentVelocity
                                            v_b = activeObjects[j].currentVelocity
                                            acc_a = activeObjects[key].acceleration
                                            acc_b = activeObjects[j].acceleration
                                            time_a = 0
                                            time_b = 0
                                            if acc_a != 0 and v_a ** 2 + 2 * acc_a * distance_a >= 0:
                                                time_a = (-v_a + math.sqrt(v_a**2 + 2 * acc_a * distance_a))/ acc_a
                                            else:   #acc_a == 0
                                                if v_a >0:
                                                    time_a = distance_a / v_a

                                            if acc_b != 0 and v_b ** 2 + 2 * acc_b * distance_b >= 0:
                                                time_b = (-v_b + math.sqrt(v_b**2 + 2 * acc_b * distance_b))/ acc_b
                                            else:   #acc_b == 0
                                                if v_b >0:
                                                    time_b = distance_b / v_b
                                            if time_a and time_b:
                                                #if car A and car B in the first timestamp,can't calculate acceleration
                                                #if timestamp == track_dictionary[key].time_stamp_ms_first == track_dictionary[j].time_stamp_ms_first:
                                                if time_a - time_b > Time_difference_max:    #b j is much faster
                                                    if(key,j) not in dependency_edges:
                                                        dependency_edges.append((key, j))
                                                elif time_b - time_a > Time_difference_max:    # a is much faster
                                                    if (j, key) not in dependency_edges:
                                                        dependency_edges.append((j,key))
                                                elif (abs(time_a - time_b) < Time_difference_max):
                                                    dependency_key_j = 1
                                                    if (key, j) not in dependency_edges:
                                                        dependency_edges.append((key, j))
                                                        dependency_edges.append((j, key))
                                    # if only car B has front car
                                    if j in calculated_pair and key not in calculated_pair:
                                        front_car_B = 0
                                        for k in calculated_pair[j]:
                                            car_point_B = lanelet2.core.BasicPoint2d(
                                                track_dictionary[k].motion_states[timestamp].x,
                                                track_dictionary[k].motion_states[timestamp].y)
                                            car_arc_B = lanelet2.geometry.toArcCoordinates(
                                                activeObjects[j].pathsWithInformation[Pb].centerline, car_point_B)
                                            if abs(
                                                    car_arc_B.distance) < Deviation_alongarcCoordinate and car_arc_B.length in Interval(
                                                    activeObjects[j].arcCoordinatesAlongPaths[Pb].length, length_area_b,
                                                    closed=False):
                                                front_car_B = 1
                                                break
                                        if front_car_B == 0:  # no car between car B and critical area
                                            v_a = activeObjects[key].currentVelocity
                                            v_b = activeObjects[j].currentVelocity
                                            acc_a = activeObjects[key].acceleration
                                            acc_b = activeObjects[j].acceleration
                                            time_a = 0
                                            time_b = 0
                                            if acc_a != 0 and v_a ** 2 + 2 * acc_a * distance_a >= 0:
                                                time_a = (-v_a + math.sqrt(v_a**2 + 2 * acc_a * distance_a))/ acc_a
                                            else:   #acc_a == 0
                                                if v_a >0:
                                                    time_a = distance_a / v_a

                                            if acc_b != 0 and v_b ** 2 + 2 * acc_b * distance_b >= 0:
                                                time_b = (-v_b + math.sqrt(v_b**2 + 2 * acc_b * distance_b))/ acc_b
                                            else:   #acc_b == 0
                                                if v_b >0:
                                                    time_b = distance_b / v_b
                                            if time_a and time_b:
                                                # if car A and car B in the first timestamp,can't calculate acceleration
                                                # if timestamp == track_dictionary[key].time_stamp_ms_first == track_dictionary[j].time_stamp_ms_first:
                                                if time_a - time_b > Time_difference_max:  # b j is much faster
                                                    if (key, j) not in dependency_edges:
                                                        dependency_edges.append((key, j))
                                                elif time_b - time_a > Time_difference_max:  # a is much faster
                                                    if (j, key) not in dependency_edges:
                                                        dependency_edges.append((j, key))
                                                elif (abs(time_a - time_b) < Time_difference_max):
                                                    dependency_key_j = 1
                                                    if (key, j) not in dependency_edges:
                                                        dependency_edges.append((key, j))
                                                        dependency_edges.append((j, key))
                                    # if both car don't have front car
                                    if key not in calculated_pair and j not in calculated_pair:
                                        v_a = activeObjects[key].currentVelocity
                                        v_b = activeObjects[j].currentVelocity
                                        acc_a = activeObjects[key].acceleration
                                        acc_b = activeObjects[j].acceleration
                                        time_a = 0
                                        time_b = 0
                                        if acc_a != 0 and v_a ** 2 + 2 * acc_a * distance_a >= 0:
                                            time_a = (-v_a + math.sqrt(v_a ** 2 + 2 * acc_a * distance_a)) / acc_a
                                        else:  # acc_a == 0
                                            if v_a > 0:
                                                time_a = distance_a / v_a

                                        if acc_b != 0 and v_b ** 2 + 2 * acc_b * distance_b >= 0:
                                            time_b = (-v_b + math.sqrt(v_b ** 2 + 2 * acc_b * distance_b)) / acc_b
                                        else:  # acc_b == 0
                                            if v_b > 0:
                                                time_b = distance_b / v_b
                                        if time_a and time_b:
                                            # if car A and car B in the first timestamp,can't calculate acceleration
                                            # if timestamp == track_dictionary[key].time_stamp_ms_first == track_dictionary[j].time_stamp_ms_first:
                                            if time_a - time_b > Time_difference_max:  # b j is much faster
                                                if (key, j) not in dependency_edges:
                                                    dependency_edges.append((key, j))
                                            elif time_b - time_a > Time_difference_max:  # a is much faster
                                                if (j, key) not in dependency_edges:
                                                    dependency_edges.append((j, key))
                                            elif (abs(time_a - time_b) < Time_difference_max):
                                                dependency_key_j = 1
                                                if (key, j) not in dependency_edges:
                                                    dependency_edges.append((key, j))
                                                    dependency_edges.append((j, key))
            #2.2  Determine dependencies based on right of way
                                #if the dependency not build yet
                                if (key,j) not in dependency_edges or (j,key) not in dependency_edges:
                                    individual_gap_a = activeObjects[key].currentVelocity * Time_gap
                                    individual_gap_b = activeObjects[j].currentVelocity * Time_gap
                                    # if the distance difference of two car to the critical area is in a threshold
                                    if abs(distance_a - distance_b) <= default_gap and \
                                            (length_area_a - activeObjects[key].arcCoordinatesAlongPaths[Pa].length) < max(default_gap,individual_gap_a) \
                                            and (length_area_b - activeObjects[j].arcCoordinatesAlongPaths[Pb].length) < max(default_gap,individual_gap_b):
                                        distance_A_to_area = 0
                                        distance_B_to_area = 0
                                        common_area_center = lanelet2.core.BasicPoint2d(first_common_area.x,first_common_area.y)
                                        #find the current conflicting lanelet for car a
                                        for ll_a in activeObjects[key].pathsWithInformation[Pa].laneletSequence:
                                            if ll_a.rightOfWay():
                                                ll_a_point = lanelet2.core.BasicPoint2d(ll_a.centerline[-1].x,ll_a.centerline[-1].y)
                                                ll_a_arcCoordinate = lanelet2.geometry.toArcCoordinates(activeObjects[key].pathsWithInformation[Pa].centerline,ll_a_point)
                                                if ll_a_arcCoordinate.distance < Deviation_alongarcCoordinate and ll_a_arcCoordinate.length >= activeObjects[key].arcCoordinatesAlongPaths[Pa].length:
                                                    ll_A = ll_a
                                                    ll_A_point = lanelet2.core.BasicPoint2d(ll_A.centerline[-1].x,ll_A.centerline[-1].y)
                                                    distance_A_to_area = lanelet2.geometry.distance(common_area_center,ll_A_point)
                                                    break
                                        #find the current conflicting lanelet for car b
                                        for ll_b in activeObjects[j].pathsWithInformation[Pb].laneletSequence:
                                            if ll_b.rightOfWay():
                                                ll_b_point = lanelet2.core.BasicPoint2d(ll_b.centerline[-1].x,ll_b.centerline[-1].y)
                                                ll_b_arcCoordinate = lanelet2.geometry.toArcCoordinates(activeObjects[j].pathsWithInformation[Pb].centerline, ll_b_point)
                                                if ll_b_arcCoordinate.distance < Deviation_alongarcCoordinate and ll_b_arcCoordinate.length >= activeObjects[j].arcCoordinatesAlongPaths[Pb].length:
                                                    ll_B = ll_b
                                                    ll_B_point = lanelet2.core.BasicPoint2d(ll_B.centerline[-1].x,ll_B.centerline[-1].y)
                                                    distance_B_to_area = lanelet2.geometry.distance(common_area_center,ll_B_point)
                                                    break

                                        #decide the final conflicting lanelet from ll_a and ll_b
                                        #compare the distance of two lanelets to the first common area
                                   # if distance_A_to_area and distance_B_to_area:
                                        if distance_A_to_area <= distance_B_to_area:
                                            ll_merge = ll_A
                                        # merge area is B
                                        else:
                                            ll_merge = ll_B
                                        # if car A(key) is in yield lanelet    right of way: B(j)
                                        if (ll_A == ll_merge and ll_A == ll_merge.rightOfWay()[0].yieldLanelets()[0]) or\
                                                (ll_B == ll_merge and ll_B == ll_merge.rightOfWay()[0].rightOfWayLanelets()[0]):
                                            ll_rightOfWay = ll_merge.rightOfWay()[0].rightOfWayLanelets()[0]
                                            ll_rightOfWay_start = lanelet2.core.BasicPoint2d(ll_rightOfWay.centerline[0].x,ll_rightOfWay.centerline[0].y)
                                            ll_rightOfWay_arcCoordinate_start = lanelet2.geometry.toArcCoordinates(activeObjects[j].pathsWithInformation[Pb].centerline,ll_rightOfWay_start)
                                            # if car A don't have front car
                                            if key not in calculated_pair:
                                                if (key, j) not in dependency_edges and (distance_b - distance_a) <= Distance_difference_max:
                                                    dependency_edges.append((key, j))
                                                if distance_a < distance_b:
                                                # if car B don't have front car and car B not pass the conflicting lanelet
                                                    if j not in calculated_pair and (j,key) not in dependency_edges:
                                                        if ll_rightOfWay_arcCoordinate_start.length > activeObjects[j].arcCoordinatesAlongPaths[Pb].length:
                                                            dependency_edges.append((j,key))
                                                # if car B have front car ,but the front car don't in the path to conflicting lanelet
                                                    if j in calculated_pair  and (j,key) not in dependency_edges:
                                                        front_car_j = 0
                                                        for k in calculated_pair[j]:
                                                            front_car_j_point = lanelet2.core.BasicPoint2d(track_dictionary[k].motion_states[timestamp].x,
                                                                                                   track_dictionary[k].motion_states[timestamp].y)
                                                            front_car_j_arcCoordinate = lanelet2.geometry.toArcCoordinates(activeObjects[j].pathsWithInformation[Pb].centerline,front_car_j_point)
                                                            if front_car_j_arcCoordinate.distance< Deviation_alongarcCoordinate and front_car_j_arcCoordinate.length < ll_rightOfWay_arcCoordinate_start.length:
                                                                front_car_j = 1
                                                        if front_car_j == 0:
                                                            if ll_rightOfWay_arcCoordinate_start.length > \
                                                                    activeObjects[j].arcCoordinatesAlongPaths[Pb].length:
                                                                dependency_edges.append((j, key))

                                            # if car A's front car has passed merge lanelet
                                            else:
                                                ll_merge_point = lanelet2.core.BasicPoint2d(
                                                    ll_merge.centerline[int(len(ll_merge.centerline) / 2)].x,
                                                    ll_merge.centerline[int(len(ll_merge.centerline) / 2)].y)
                                                ll_merge_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                    activeObjects[key].pathsWithInformation[Pa].centerline,
                                                    ll_merge_point)
                                                front_car = 0
                                                for k in calculated_pair[key]:
                                                    car_point = lanelet2.core.BasicPoint2d(
                                                        track_dictionary[k].motion_states[timestamp].x,
                                                        track_dictionary[k].motion_states[timestamp].y)
                                                    front_car_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                        activeObjects[key].pathsWithInformation[Pa].centerline,
                                                        car_point)
                                                    if front_car_arcCoordinate.distance < Deviation_alongarcCoordinate and front_car_arcCoordinate.length < ll_merge_arcCoordinate.length:
                                                        front_car = 1
                                                        break
                                                if front_car == 0:
                                                    if (key, j) not in dependency_edges and (distance_b - distance_a) <= Distance_difference_max:
                                                        dependency_edges.append((key, j))
                                                    if distance_a < distance_b:
                                                    # if car B don't have front car and car B not pass the conflicting lanelet
                                                        if j not in calculated_pair and (j, key) not in dependency_edges:
                                                            if ll_rightOfWay_arcCoordinate_start.length > \
                                                                    activeObjects[j].arcCoordinatesAlongPaths[Pb].length:
                                                                dependency_edges.append((j, key))
                                                    # if car B have front car ,but the front don't in the path to conflicting lanelet
                                                        if j in calculated_pair  and (j,key) not in dependency_edges:
                                                            front_car_j = 0
                                                            for k in calculated_pair[j]:
                                                                front_car_j_point = lanelet2.core.BasicPoint2d(
                                                                    track_dictionary[k].motion_states[timestamp].x,
                                                                    track_dictionary[k].motion_states[timestamp].y)
                                                                front_car_j_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                                    activeObjects[j].pathsWithInformation[
                                                                        Pb].centerline, front_car_j_point)
                                                                if front_car_j_arcCoordinate.distance < Deviation_alongarcCoordinate and front_car_j_arcCoordinate.length <ll_rightOfWay_arcCoordinate_start.length:
                                                                    front_car_j = 1
                                                                    break
                                                            if front_car_j == 0:
                                                                if ll_rightOfWay_arcCoordinate_start.length > \
                                                                        activeObjects[j].arcCoordinatesAlongPaths[
                                                                            Pb].length:
                                                                    dependency_edges.append((j, key))
                                        #if car B (j)is in yield lanelet      j is yield    key(a) has right
                                        else:
                                            ll_rightOfWay = ll_merge.rightOfWay()[0].rightOfWayLanelets()[0]
                                            ll_rightOfWay_start = lanelet2.core.BasicPoint2d(
                                                ll_rightOfWay.centerline[0].x, ll_rightOfWay.centerline[0].y)
                                            ll_rightOfWay_arcCoordinate_start = lanelet2.geometry.toArcCoordinates(
                                                activeObjects[key].pathsWithInformation[Pa].centerline,
                                                ll_rightOfWay_start)
                                            # if B don't have front car
                                            if j not in calculated_pair:
                                                if (j,key) not in dependency_edges and (distance_a - distance_b) <= Distance_difference_max:
                                                    dependency_edges.append((j, key))
                                                if distance_a > distance_b:
                                                # if car A don't have front car and car A not pass the conflicting lanelet
                                                    if key not in calculated_pair and  (key, j) not in dependency_edges :
                                                        if ll_rightOfWay_arcCoordinate_start.length > activeObjects[key].arcCoordinatesAlongPaths[Pa].length:
                                                            dependency_edges.append((key, j))
                                             # if car A have front car ,but the front don't in the path to conflichting lanelet
                                                    if key in calculated_pair  and (key,j) not in dependency_edges:
                                                        front_car_key = 0
                                                        for k in calculated_pair[key]:
                                                            front_car_key_point = lanelet2.core.BasicPoint2d(
                                                                track_dictionary[k].motion_states[timestamp].x,
                                                                track_dictionary[k].motion_states[timestamp].y)
                                                            front_car_key_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                                activeObjects[key].pathsWithInformation[Pa].centerline,
                                                                front_car_key_point)
                                                            if front_car_key_arcCoordinate.distance < Deviation_alongarcCoordinate and front_car_key_arcCoordinate.length < ll_rightOfWay_arcCoordinate_start.length:
                                                                front_car_key = 1
                                                        if front_car_key == 0 :
                                                            if ll_rightOfWay_arcCoordinate_start.length > \
                                                                    activeObjects[key].arcCoordinatesAlongPaths[Pa].length:
                                                                dependency_edges.append((key, j))
                                            # if car B don't have front car before the conflicting lanelet
                                            else:
                                                ll_merge_point = lanelet2.core.BasicPoint2d(
                                                    ll_merge.centerline[int(len(ll_merge.centerline) / 2)].x,
                                                    ll_merge.centerline[int(len(ll_merge.centerline) / 2)].y)
                                                ll_merge_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                    activeObjects[j].pathsWithInformation[Pb].centerline,
                                                    ll_merge_point)
                                                front_car = 0
                                                for k in calculated_pair[j]:
                                                    car_point = lanelet2.core.BasicPoint2d(
                                                        track_dictionary[k].motion_states[timestamp].x,
                                                        track_dictionary[k].motion_states[timestamp].y)
                                                    front_car_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                        activeObjects[j].pathsWithInformation[Pb].centerline,
                                                        car_point)
                                                    if front_car_arcCoordinate.distance < Deviation_alongarcCoordinate and front_car_arcCoordinate.length < ll_merge_arcCoordinate.length:
                                                        front_car = 1
                                                        break
                                                if front_car == 0:
                                                    if (j, key) not in dependency_edges and (distance_a - distance_b) <= Distance_difference_max:
                                                        dependency_edges.append((j, key))
                                                    # if car A don't have front car
                                                    if distance_a > distance_b:
                                                        if key not in calculated_pair and (key, j) not in dependency_edges:
                                                            if ll_rightOfWay_arcCoordinate_start.length > \
                                                                    activeObjects[key].arcCoordinatesAlongPaths[Pa].length:
                                                                dependency_edges.append((key, j))
                                                    # if car A have front car ,but the front don't in the path to critical area
                                                        if key in calculated_pair and (key, j) not in dependency_edges:
                                                            front_car_key = 0
                                                            for k in calculated_pair[key]:
                                                                front_car_key_point = lanelet2.core.BasicPoint2d(
                                                                    track_dictionary[k].motion_states[timestamp].x,
                                                                    track_dictionary[k].motion_states[timestamp].y)
                                                                front_car_key_arcCoordinate = lanelet2.geometry.toArcCoordinates(
                                                                    activeObjects[key].pathsWithInformation[Pa].centerline,
                                                                    front_car_key_point)
                                                                if front_car_key_arcCoordinate.distance < Deviation_alongarcCoordinate and front_car_key_arcCoordinate.length < ll_rightOfWay_arcCoordinate_start.length:
                                                                    front_car_key = 1
                                                                    break
                                                            if front_car_key == 0:
                                                                if ll_rightOfWay_arcCoordinate_start.length > \
                                                                        activeObjects[key].arcCoordinatesAlongPaths[
                                                                            Pa].length:
                                                                    dependency_edges.append((key, j))

            if not paired or key not in paired:
                paired[key] = car_list  # key is center car id,  value is the other cars list, which it depends on

    return dependency_node,dependency_edges

