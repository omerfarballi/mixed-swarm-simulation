import ipc
import dijkstar

# ===================================================================================================================
road_edge_to_edge_cache = {}
road_nodes_to_edge_index = None
road_graph = None
road_edges_by_start_node = None
closest_road_point_memoization = {}
ugv_victim_to_handle = {}


# ===================================================================================================================
def main_loop():
    for i in range(ipc.UgvData.item_count()):
        ugv_main_loop(i)
    victim_sending_loop()


# ===================================================================================================================
def ugv_main_loop(ugv_index):
    ugv = ipc.UgvData.id(ugv_index)

    ipc.CmdUgv.set_fire_hose_pressure(ugv, 0)
    ipc.CmdUgv.set_throttle(ugv, 0.0)
    ipc.CmdUgv.set_brake(ugv, 1.0)

    r = ugv_engage_station(ugv, True)
    r |= ugv_engage_station(ugv, False)
    if r:
        return

    if ipc.UgvData.passenger_capacity(ugv_index) == 0 or ugv % 2 == 0:  # ultra simple task sharing
        if ugv_engage_any_fire(ugv):
            return
    if ugv_engage_any_victim(ugv):
        return
    if ugv_engage_destination(ugv):
        return


# ===================================================================================================================
def ugv_engage_any_fire(ugv):
    fire_count = ipc.FireData.item_count()
    if fire_count == 0:
        return False
    fire_index = ugv % fire_count  # randomization by ugv-id
    fire_id = ipc.FireData.id(fire_index)
    ugv_engage_fire(ugv, fire_id)
    return True


# ===================================================================================================================
def ugv_engage_station(ugv, for_fuel):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)
    resource_value = ipc.UgvData.fuel_value(ugv_index) if for_fuel else ipc.UgvData.water_value(ugv_index)
    resource_capacity = ipc.UgvData.fuel_capacity(ugv_index) if for_fuel else ipc.UgvData.water_capacity(ugv_index)
    resource_ratio = resource_value / resource_capacity
    attached_this = ipc.UgvData.attached_fuel_station(
        ugv_index) != 0 if for_fuel else ipc.UgvData.attached_water_station(ugv_index) != 0
    attached_any = ipc.UgvData.attached_fuel_station(ugv_index) != 0 or ipc.UgvData.attached_water_station(
        ugv_index) != 0
    set_attached = ipc.CmdUgv.set_attach_to_station_for_fuel if for_fuel else ipc.CmdUgv.set_attach_to_station_for_water

    if attached_this:
        if resource_ratio >= 0.99:
            set_attached(ugv, 0)
        return True

    if resource_ratio > 0.8:
        return False

    if attached_any:
        attached_station_id = ipc.UgvData.attached_water_station(
            ugv_index) if for_fuel else ipc.UgvData.attached_fuel_station(ugv_index)
        attached_station_index = find_index_by_id(ipc.StationData, attached_station_id)
        if station_ready_for_transfer(attached_station_index, for_fuel):
            if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STOPPED:
                set_attached(ugv, attached_station_id)
        return True

    if resource_ratio > 0.4:
        return False

    station_index = find_suitable_station_index_for_ugv(ugv_index)
    if station_index is None:
        return True

    station_id = ipc.StationData.id(station_index)
    update_ugv_location(ugv, ipc.StationData.location(station_index))
    if not ugv_inside_station(ugv_index, station_index):
        return True
    if ipc.UgvData.motor_state(ugv_index) != ipc.MOTOR_STATE__STOPPED:
        return True
    set_attached(ugv, station_id)
    return True


# ===================================================================================================================
def find_suitable_station_index_for_ugv(ugv_index):
    need_fuel = ipc.UgvData.fuel_value(ugv_index) / ipc.UgvData.fuel_capacity(ugv_index) <= 0.4
    need_water = ipc.UgvData.water_value(ugv_index) / ipc.UgvData.water_capacity(ugv_index) <= 0.4
    for i in range(ipc.StationData.item_count()):
        if ipc.StationData.is_surface(i):
            continue
        if need_fuel and not station_ready_for_transfer(i, True):
            continue
        if need_water and not station_ready_for_transfer(i, False):
            continue
        return i
    return None


# ===================================================================================================================
def station_ready_for_transfer(station_index, for_fuel):
    max_vehicle = ipc.StationData.vehicle_capacity(station_index)
    curr_vehicle = ipc.StationData.attached_vehicle_count(station_index)
    rem_vehicle = max_vehicle - curr_vehicle
    if rem_vehicle <= 0:
        return False
    curr_resource = ipc.StationData.fuel_value(station_index) if for_fuel else ipc.StationData.water_value(
        station_index)
    cap_resource = ipc.StationData.fuel_capacity(station_index) if for_fuel else ipc.StationData.water_capacity(
        station_index)
    if cap_resource <= 1.0 or curr_resource <= 1.0:
        return False
    return (curr_resource / cap_resource) >= 0.4


# ===================================================================================================================
def ugv_inside_station(ugv_index, station_index):
    ugv_pos = ipc.UgvData.position(ugv_index)
    station_pos = ipc.StationData.location(station_index)
    station_range = ipc.StationData.range(station_index)
    distance = magnitude(delta(ugv_pos, station_pos))
    return distance <= station_range


# ===================================================================================================================
def victim_sending_loop():
    for i in range(ipc.VictimData.item_count()):
        if ipc.VictimData.victim_state(i) != ipc.VICTIM_STATE__STANDING:
            continue
        destination_index = victim_in_sending_range(i)
        if destination_index is not None:
            victim_id = ipc.VictimData.id(i)
            ipc.CmdVictim.set_command(victim_id, ipc.EVC_COMMAND__NO_OP)
            ipc.CmdVictim.set_arg0(victim_id, ipc.DestinationData.id(destination_index))
            ipc.CmdVictim.set_command(victim_id, ipc.EVC_COMMAND__SEND)


# ===================================================================================================================
def ugv_engage_destination(ugv):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)

    if ipc.UgvData.victim_count(ugv_index) <= 0:
        return False

    di = first_suitable_destination_id()
    if di is None:
        return False
    destination_pos = ipc.DestinationData.location(di)
    my_pos = ipc.UgvData.position(ugv_index)
    update_ugv_location(ugv, destination_pos)
    if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STOPPED:
        victim_indexes = victim_indexes_of_vehicle(ipc.VEHICLE_TYPE__UGV, ugv)
        for victim_index in victim_indexes:
            vid = ipc.VictimData.id(victim_index)
            ipc.CmdVictim.set_command(vid, ipc.EVC_COMMAND__NO_OP)
            ipc.CmdVictim.set_arg0(vid, my_pos[0])
            ipc.CmdVictim.set_arg1(vid, my_pos[2])
            ipc.CmdVictim.set_command(vid, ipc.EVC_COMMAND__UNLOAD)
    return True


# ===================================================================================================================
def victim_indexes_of_vehicle(vehicle_type, vehicle_id):
    res = []
    for i in range(ipc.VictimData.item_count()):
        if vehicle_type != ipc.VictimData.current_vehicle_type(i):
            continue
        if vehicle_id != ipc.VictimData.current_vehicle_id(i):
            continue
        res.append(i)
    return res


# ===================================================================================================================
def ugv_engage_any_victim(ugv):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)
    if ipc.UgvData.victim_count(ugv_index) >= ipc.UgvData.passenger_capacity(ugv_index):
        return False
    if ipc.VictimData.item_count() == 0:
        return False
    if not (ugv in ugv_victim_to_handle):
        new_target = closest_loadable_victim_id(ipc.UgvData.position(ugv_index))
        if new_target is None:
            if ugv in ugv_victim_to_handle:
                del ugv_victim_to_handle[ugv]
            return False
        else:
            ugv_victim_to_handle[ugv] = new_target

    target_victim_id = ugv_victim_to_handle[ugv]
    target_victim_index = find_index_by_id(ipc.VictimData, target_victim_id)
    if (target_victim_index is None) or (
            ipc.VictimData.victim_state(target_victim_index) == ipc.VICTIM_STATE__ON_BOARD) \
            or victim_in_sending_range(target_victim_index) is not None:
        del ugv_victim_to_handle[ugv]
        return False

    ugv_engage_victim(ugv, target_victim_id)
    return True


# ===================================================================================================================
def victim_in_sending_range(victim_index):
    pos = ipc.VictimData.location(victim_index)
    ran = ipc.VictimData.calling_range(victim_index)
    for i in range(ipc.DestinationData.item_count()):
        destination_pos = ipc.DestinationData.location(i)
        mag = magnitude(delta(destination_pos, pos))
        if mag <= ran:
            return i
    return None


# ===================================================================================================================
def closest_loadable_victim_id(position):
    selected_i = None
    selected_mag = None
    for i in range(ipc.VictimData.item_count()):
        state = ipc.VictimData.victim_state(i)
        if state == ipc.VICTIM_STATE__ARRIVING or state == ipc.VICTIM_STATE__ON_BOARD:
            continue
        if victim_in_sending_range(i) is not None:
            continue
        pos = ipc.VictimData.location(i)
        mag = square_magnitude(delta(position, pos))
        if selected_mag is None or mag < selected_mag:
            selected_i = i
            selected_mag = mag
    if selected_i is None:
        return None
    return ipc.VictimData.id(selected_i)


# ===================================================================================================================
def first_suitable_destination_id():
    for i in range(ipc.DestinationData.item_count()):
        if ipc.DestinationData.current_capacity(i) == 0:
            continue
        return i
    return None


# ===================================================================================================================
def ugv_engage_fire(ugv, fire):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)
    fire_index = find_index_by_id(ipc.FireData, fire)
    if fire_index is None:
        ipc.CmdUgv.set_fire_hose_pressure(ugv, 0)
        ipc.CmdUgv.set_throttle(ugv, 0)
        ipc.CmdUgv.set_brake(ugv, 1)
        return
    fire_location = ipc.FireData.location(fire_index)
    update_ugv_location(ugv, fire_location)

    min_range = ipc.UgvData.fire_hose_range_min(ugv_index)
    max_range = ipc.UgvData.fire_hose_range_max(ugv_index)
    current_pos = ipc.UgvData.position(ugv_index)

    d = delta(fire_location, current_pos)
    mag = magnitude(d)
    if mag > max_range or mag < min_range:
        ipc.CmdUgv.set_fire_hose_pressure(ugv, 0)
        return
    ipc.CmdUgv.set_fire_hose_direction(ugv, d)
    ipc.CmdUgv.set_fire_hose_pressure(ugv, 1e4)


# ===================================================================================================================
def ugv_engage_victim(ugv, victim):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)
    victim_index = find_index_by_id(ipc.VictimData, victim)
    if victim_index is None:
        return
    victim_location = ipc.VictimData.location(victim_index)
    my_location = ipc.UgvData.position(ugv_index)
    delta_distance = delta(victim_location, my_location)
    victim_state = ipc.VictimData.victim_state(victim_index)
    delta_mag = magnitude(delta_distance)
    if victim_state == ipc.VICTIM_STATE__ARRIVING:
        return
    if victim_state == ipc.VICTIM_STATE__STANDING:
        if delta_mag <= ipc.VictimData.calling_range(victim_index):
            ipc.CmdVictim.set_command(victim, ipc.EVC_COMMAND__NO_OP)
            ipc.CmdVictim.set_arg0(victim, ipc.VEHICLE_TYPE__UGV)
            ipc.CmdVictim.set_arg1(victim, ugv)
            ipc.CmdVictim.set_command(victim, ipc.EVC_COMMAND__CALL)
        else:
            ipc.CmdVictim.set_command(victim, ipc.EVC_COMMAND__NO_OP)
    if victim_state == ipc.VICTIM_STATE__CALLING or victim_state == ipc.VICTIM_STATE__ON_BOARD:
        if ipc.VictimData.current_vehicle_type(victim_index) != ipc.VEHICLE_TYPE__UGV:
            return
        if ipc.VictimData.current_vehicle_id(victim_index) != ugv:
            return

    if victim_state == ipc.VICTIM_STATE__STANDING or victim_state == ipc.VICTIM_STATE__CALLING:
        update_ugv_location(ugv, ipc.VictimData.location(victim_index))

    if victim_state == ipc.VICTIM_STATE__CALLING:
        if delta_mag <= ipc.VictimData.loading_range(victim_index):
            ipc.CmdVictim.set_command(victim, ipc.EVC_COMMAND__NO_OP)
            ipc.CmdVictim.set_arg0(victim, ipc.VEHICLE_TYPE__UGV)
            ipc.CmdVictim.set_arg1(victim, ugv)
            ipc.CmdVictim.set_command(victim, ipc.EVC_COMMAND__LOAD)


# ===================================================================================================================
def destination_index_with_max_capacity():
    max_cap = None
    max_id = None
    for i in range(ipc.DestinationData.item_count()):
        cap = ipc.DestinationData.current_capacity(i)
        if max_cap is None or cap > max_cap:
            max_cap = cap
            max_id = i
    if max_cap <= 0:
        return None
    return max_id


# ===================================================================================================================
def update_ugv_location(ugv, location):
    edge, time = closest_road_point(location)
    update_ugv_transform(ugv, edge, time)


# ===================================================================================================================
def update_ugv_transform(ugv, target_edge, target_time):
    ugv_index = find_index_by_id(ipc.UgvData, ugv)
    if ugv_index is None:
        return

    current_edge = ipc.UgvData.edge(ugv_index)
    current_time = ipc.UgvData.time(ugv_index)

    if current_edge == target_edge and almost_same_time(current_time, target_time):
        if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STARTED:
            ipc.CmdUgv.set_motor_running(ugv, False)
        ipc.CmdUgv.set_throttle(ugv, 0)
        ipc.CmdUgv.set_brake(ugv, 1)
        return

    if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STARTING:
        return
    if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STOPPING:
        return
    if ipc.UgvData.motor_state(ugv_index) == ipc.MOTOR_STATE__STOPPED:
        ipc.CmdUgv.set_motor_running(ugv, True)
        return

    new_turn_choice = calculate_turn_choice_by_shortest_path(current_edge, target_edge)
    if new_turn_choice is not None:
        ipc.CmdUgv.set_turn_choice(ugv, new_turn_choice)

    if current_edge != target_edge:
        if ipc.UgvData.forward_gear(ugv_index):
            ipc.CmdUgv.set_throttle(ugv, default_throttle_by_shortest_path(current_edge, target_edge))
            ipc.CmdUgv.set_brake(ugv, 0.0 if magnitude(ipc.UgvData.velocity(ugv_index)) <= 12 else 0.5)
        else:
            ipc.CmdUgv.set_throttle(ugv, 0.0)
            ipc.CmdUgv.set_brake(ugv, 1.0)
            ipc.CmdUgv.set_backward_gear(ugv, False)
    else:
        update_ugv_time_control_in_same_edge(ugv, ugv_index, target_time)


# ===================================================================================================================
def clamp01(v):
    if v < 0:
        return 0
    if v > 1:
        return 1
    return v


# ===================================================================================================================
def delta(a, b):
    if len(a) == 2:
        return [a[0] - b[0], a[1] - b[1]]
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


# ===================================================================================================================
def hermite(p0, m0, p1, m1, time):
    time = clamp01(time)
    t2 = time * time
    t3 = t2 * time
    e1 = 2 * t3 - 3 * t2 + 1
    e2 = t3 - 2 * t2 + time
    e3 = -2 * t3 + 3 * t2
    e4 = t3 - t2
    return e1 * p0 + e2 * m0 + e3 * p1 + e4 * m1


# ===================================================================================================================
def calc_hermite_value(edge, time):
    p0 = ipc.RoadNode.position(ipc.RoadEdge.start_node(edge))
    p1 = ipc.RoadNode.position(ipc.RoadEdge.end_node(edge))
    m0 = ipc.RoadEdge.start_tangent(edge)
    m1 = ipc.RoadEdge.end_tangent(edge)
    x = hermite(p0[0], m0[0], p1[0], m1[0], time)
    y = hermite(p0[1], m0[1], p1[1], m1[1], time)
    z = hermite(p0[2], m0[2], p1[2], m1[2], time)
    return [x, y, z]


# ===================================================================================================================
def closest_road_point(position):
    if tuple(position) in closest_road_point_memoization:
        return closest_road_point_memoization[tuple(position)]

    selected_i = None
    selected_distance = None
    selected_time = 0

    for i in range(ipc.RoadEdge.item_count()):
        a = calc_hermite_value(i, 0.25)
        b = calc_hermite_value(i, 0.50)
        c = calc_hermite_value(i, 0.75)
        sqrt_dist1 = square_magnitude(delta(position, a))
        sqrt_dist2 = square_magnitude(delta(position, b))
        sqrt_dist3 = square_magnitude(delta(position, c))
        min_distance = min(sqrt_dist1, sqrt_dist2, sqrt_dist3)
        if selected_i is None or min_distance < selected_distance:
            selected_i = i
            selected_distance = min_distance
            continue

    selected_distance = 1e7

    for i in range(100):
        t = i / 100
        dist = square_magnitude(delta(position, calc_hermite_value(selected_i, t)))
        if dist < selected_distance:
            selected_distance = dist
            selected_time = t

    closest_road_point_memoization[tuple(position)] = selected_i, selected_time
    return selected_i, selected_time


# ===================================================================================================================
def default_throttle_by_shortest_path(current_edge, target_edge):
    n = len(get_shortest_path_edge_to_edge(current_edge, target_edge).nodes)
    if n >= 10:
        return 0.7
    if n >= 5:
        return 0.1
    return 0.05


# ===================================================================================================================
def update_ugv_time_control_in_same_edge(ugv_id, ugv_index, target_time):
    current_time = ipc.UgvData.time(ugv_index)
    gear = ipc.UgvData.forward_gear(ugv_index)
    vel_magnitude = magnitude(ipc.UgvData.velocity(ugv_index))
    if almost_same_time(current_time, target_time):
        ipc.CmdUgv.set_throttle(ugv_id, 0)
        ipc.CmdUgv.set_brake(ugv_id, 1)
        return
    if vel_magnitude > 3.0:
        ipc.CmdUgv.set_throttle(ugv_id, 0)
        ipc.CmdUgv.set_brake(ugv_id, 1)
        return
    if current_time < target_time:
        if gear:
            ipc.CmdUgv.set_throttle(ugv_id, 0.1)
            ipc.CmdUgv.set_brake(ugv_id, 0)
        else:
            ipc.CmdUgv.set_throttle(ugv_id, 0)
            ipc.CmdUgv.set_brake(ugv_id, 1)
            if vel_magnitude <= 0.001:
                ipc.CmdUgv.set_backward_gear(ugv_id, False)
    else:
        if gear:
            ipc.CmdUgv.set_throttle(ugv_id, 0)
            ipc.CmdUgv.set_brake(ugv_id, 1)
            if vel_magnitude <= 0.001:
                ipc.CmdUgv.set_backward_gear(ugv_id, True)
        else:
            ipc.CmdUgv.set_throttle(ugv_id, 0.1)
            ipc.CmdUgv.set_brake(ugv_id, 0)


# ===================================================================================================================
def float_eq(a, b, tolerance):
    return abs(a - b) <= tolerance


# ===================================================================================================================
def almost_same_time(a, b):
    return float_eq(a, b, 0.001)


# ===================================================================================================================
def find_index_by_id(clazz, item_id):
    item_count = clazz.item_count()
    for i in range(item_count):
        if clazz.id(i) == item_id:
            return i
    return None


# ===================================================================================================================
def square_magnitude(vector):
    res = vector[0] ** 2 + vector[1] ** 2
    if len(vector) > 2:
        res += vector[2] ** 2
    return res


# ===================================================================================================================
def magnitude(vector):
    return square_magnitude(vector) ** 0.5


# ===================================================================================================================
def get_edges_by_start_node(start_node):
    global road_edges_by_start_node
    if road_edges_by_start_node is None:
        road_edges_by_start_node = {}
        for i in range(ipc.RoadEdge.item_count()):
            s = ipc.RoadEdge.start_node(i)
            for j in range(ipc.RoadNode.item_count()):
                if j == s:
                    if not (s in road_edges_by_start_node):
                        road_edges_by_start_node[s] = []
                    road_edges_by_start_node[s].append(i)
    return road_edges_by_start_node[start_node]


# ===================================================================================================================
def calculate_adjacent_edges(edge):
    my_end = ipc.RoadEdge.end_node(edge)
    return get_edges_by_start_node(my_end)


# ===================================================================================================================
def find_turn_choice(from_edge, to_edge):
    adj = calculate_adjacent_edges(from_edge)
    if adj[0] == to_edge:
        return 0
    if len(adj) > 1 and adj[1] == to_edge:
        return 1
    return 0


# ===================================================================================================================
def calculate_turn_choice_by_shortest_path(current_edge, target_edge):
    shortest_path = get_shortest_path_edge_to_edge(current_edge, target_edge)
    if shortest_path.nodes[0] != ipc.RoadEdge.start_node(current_edge):
        raise "?"
    if len(shortest_path.nodes) < 3:
        return None
    next1 = shortest_path.nodes[1]
    next2 = shortest_path.nodes[2]
    next_edge = edge_index_by_terminal_nodes(next1, next2)
    return find_turn_choice(current_edge, next_edge)


# ===================================================================================================================
def edge_index_by_terminal_nodes(start_node, end_node):
    global road_nodes_to_edge_index
    if not (road_nodes_to_edge_index is None):
        return road_nodes_to_edge_index[(start_node, end_node)]
    road_nodes_to_edge_index = {}
    for i in range(ipc.RoadEdge.item_count()):
        road_nodes_to_edge_index[(ipc.RoadEdge.start_node(i), ipc.RoadEdge.end_node(i))] = i
    return road_nodes_to_edge_index[(start_node, end_node)]


# ===================================================================================================================
def calculate_road_graph_if_necessary():
    global road_graph
    if not (road_graph is None):
        return
    road_graph = dijkstar.Graph()
    edge_count = ipc.RoadEdge.item_count()
    for i in range(edge_count):
        start_node = ipc.RoadEdge.start_node(i)
        end_node = ipc.RoadEdge.end_node(i)
        edge_length = ipc.RoadEdge.length(i)
        road_graph.add_edge(start_node, end_node, edge_length)


# ===================================================================================================================
def calculate_shortest_path_node_to_node(from_node, to_node):
    calculate_road_graph_if_necessary()
    return dijkstar.find_path(road_graph, from_node, to_node)


# ===================================================================================================================
def calculate_shortest_path_edge_to_edge(from_edge, to_edge):
    start_node = ipc.RoadEdge.start_node(from_edge)
    end_node = ipc.RoadEdge.end_node(to_edge)
    return calculate_shortest_path_node_to_node(start_node, end_node)


# ===================================================================================================================
def get_shortest_path_edge_to_edge(from_edge, to_edge):
    if (from_edge, to_edge) in road_edge_to_edge_cache:
        return road_edge_to_edge_cache[(from_edge, to_edge)]
    res = calculate_shortest_path_edge_to_edge(from_edge, to_edge)
    road_edge_to_edge_cache[(from_edge, to_edge)] = res
    return res
