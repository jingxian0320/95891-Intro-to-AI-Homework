from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import csv
import datetime

def get_locations(loc_filepath = 'locations.txt'):
    '''Read the file and return a list of all locations
    '''
    return [line.rstrip('\n') for line in open(loc_filepath)]


def get_distance_matrix(dist_filepath = 'distances.csv'):
    '''
    Read the file and return a list of lists or a 2D array with 
    distances such that dist_matrix[loc1][loc2] returns the distance from loc1 to loc2
    e.g. dist_matrix = [[0,4,9],
                        [4,0,8],
                        [9,8,0]] for 3  sample locations
    '''
    return [[int(x) for x in line] for line in csv.reader(open(dist_filepath, 'r'), delimiter=',')]


def create_data_model(distances, num_vehicles = 1, depot = 0):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = distances
    data['num_vehicles'] = num_vehicles
    data['depot'] = depot
    return data


def print_solution(manager, routing, assignment,location_names):
    """Prints assignment on console."""
    print('Objective: {} miles'.format(assignment.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = 'Route for vehicle 0:\n'
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += ' {} ->'.format(location_names[manager.IndexToNode(index)])
        previous_index = index
        index = assignment.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += ' {}\n'.format(location_names[manager.IndexToNode(index)])
    plan_output += 'Route distance: {} miles\n'.format(route_distance)
    print(plan_output)

def get_routine(manager, routing, assignment):
    result = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        result.append(manager.IndexToNode(index))
        index = assignment.Value(routing.NextVar(index))
    result.append(manager.IndexToNode(index))
    return result
    
def main():
    """Entry point of the program."""
    #Get distance matrix and location list
    dist_filepath =  "distances.csv"
    loc_filepath =  "locations.txt"
    dist_matrix = get_distance_matrix(dist_filepath)
    location_names = get_locations(loc_filepath)
    start_time = datetime.datetime.now()
    
    
    # Instantiate the data problem.
    data = create_data_model(dist_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(
        len(data['distance_matrix']), data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)


    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    assignment = routing.SolveWithParameters(search_parameters)
    print ("Calculation Completed: " + str(datetime.datetime.now() - start_time))
    # Print solution on console.
    if assignment:
        print_solution(manager, routing, assignment,location_names)
        import HW1p3
        HW1p3.plot_routine(get_routine(manager, routing, assignment))


if __name__ == '__main__':
    main()
    
