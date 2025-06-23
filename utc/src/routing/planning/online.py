from utc.src.routing.planning.mode import (
    Mode, PddlOptions, Scenario, Graph, RoadNetwork,
    ProblemGenerator, ResultGenerator, Parser
)
from utc.src.routing.pddl.pddl_episode import PddlEpisode, PddlProblem, PddlResult
from utc.src.routing.planning.scheduler import Scheduler, SumoVehicle, SumoRoute, VehicleStats
from utc.src.simulator.simulation import Simulation, traci
from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
from xml.etree.ElementTree import Element
from math import ceil
from copy import deepcopy
from typing import Optional, List, Set, Tuple, Dict
import time

class Online(Mode):
    """ Class representing 'online' mode of planning """

    def __init__(self, options: PddlOptions):
        self.sub_graphs: List[Graph] = []
        self.sub_graph_names: List[str] = ["lust_lime", "lust_orange", "lust_red"]
        # self.sub_graph_names: List[str] = ["DCC_red", "DCC_orange", "DCC_green"]
        super().__init__(options)
        self.low: int = 5
        self.timeout: int = 9
        self.window: int = 10
        self.high: int = 18
        self.scheduler: Scheduler = Scheduler(
            [graph.road_network for graph in self.sub_graphs],
            (self.low, self.timeout, self.high)
        )

    def initialize(self) -> bool:
        if self.options is None:
            print("Received invalid PDDL options!")
            return False
        # Initialize scenarios
        self.scenario = Scenario(self.options.init.scenario)
        self.new_scenario = Scenario(self.options.init.new_scenario, True)
        if not self.new_scenario.scenario_dir.initialize_dir(pddl=True, info_dir=True):
            return False
        elif not self.scenario.exists():
            return False
        # Initialize graph
        self.graph: Graph = Graph(RoadNetwork())
        if not self.graph.loader.load_map(self.scenario.config_file.get_network()):
            return False
        # Assign starting travel time to edges (free-flow travel time)
        for edge in self.graph.road_network.get_edge_list():
            edge.attributes["travelTime"] = edge.get_travel_time()
        # Initialize sub-graph, if there is any
        self.problem_generator = []
        self.parser = []
        for sub_graph_name in self.sub_graph_names:
            self.sub_graphs.append(Graph(RoadNetwork()))
            assert (self.sub_graphs[-1].loader.load_map(sub_graph_name))
            # Copy travel times to regions to be updated automatically
            for edge in self.sub_graphs[-1].road_network.get_edge_list():
                edge.attributes = self.graph.road_network.get_edge(edge.id).attributes
            self.problem_generator.append(
                ProblemGenerator(self.new_scenario, self.options.network, self.graph, self.sub_graphs[-1])
            )
            self.parser.append(Parser(self.graph, self.sub_graphs[-1]))
        self.result_generator = ResultGenerator(self.new_scenario.scenario_dir)
        return True

    def generate_episodes(self) -> List[PddlEpisode]:
        episodes: List[PddlEpisode] = []
        step_length: float = self.scenario.config_file.get_step_length()
        steps: int = int(self.window / step_length)
        planning_steps: int = int(self.timeout / self.scenario.config_file.get_step_length())
        assert (steps == 20 and planning_steps == 18)
        counter: int = 0
        planned_vehicles: List[Dict[Element, Element]] = []
        # planned_vehicles: List[str] = []
        total: VehicleStats = VehicleStats()
        options: dict = {
            "-W": "",
            # "--duration-log.statistics": "true",
            # "--statistic-output": "test.xml"
        }
        time_steps: int = 0
        assigned_routes: bool = False
        with Simulation(self.scenario.config_file, options=options) as simulation:
            while simulation.is_running():
                print(f"---------- Current time: {simulation.get_time(False)}, step: {counter} ----------")
                assigned_routes = False
                # Advance simulation by maximal timeout for solvers
                for planning_step in range(1, planning_steps + 1):
                    simulation.step()
                    self.scheduler.step(simulation)
                    # Check if results were generated sooner, assign new routes if possible asap
                    if planning_step == time_steps:
                        assigned_routes = True
                        # Check for results in planner, check which vehicles can have routes assigned to them
                        can_assign: List[str] = self.scheduler.assign_planned(
                            [vehicle.attrib["id"] for entry in planned_vehicles for vehicle in entry],
                            self.graph.road_network
                        )
                        self.assign_routes(planned_vehicles, set(can_assign))
                # Check if simulation is still running
                if not simulation.is_running():
                    break
                # Assign routes if results took the maximum possible time to generate
                if not assigned_routes:
                    # Check for results in planner, check which vehicles can have routes assigned to them
                    can_assign: List[str] = self.scheduler.assign_planned(
                        [vehicle.attrib["id"] for entry in planned_vehicles for vehicle in entry],
                        self.graph.road_network
                    )
                    self.assign_routes(planned_vehicles, set(can_assign))
                # Continue simulation to another break point (window)
                for _ in range(steps - planning_steps):
                    simulation.step()
                    self.scheduler.step(simulation)
                # Check if simulation is still running
                if not simulation.is_running():
                    break
                # Compute ETA's of vehicles, generate planning problems for those within range (low <= eta <= high)
                self.scheduler.update_travel_time(self.graph.road_network)
                # for interval in edge_data.root.findall("interval"):
                scheduled: List[SumoVehicle] = self.scheduler.compute_etas(self.graph.road_network)
                # print(self.scheduler.stats)
                total += self.scheduler.stats
                self.scheduler.stats.reset()
                # planned_vehicles = [vehicle.id for vehicle in scheduled]
                # Generate problems & results, pretend this is asynchronous, so results will be ready next step
                if scheduled:
                    now: float = time.time()
                    episode: List[PddlEpisode] = self.generate_episode(scheduled, simulation)
                    # We are guaranteed to solve results, since we kill processes with timeout
                    time_steps = min(self.get_steps(round(time.time() - now, 1), step_length), planning_steps)
                    print(f"Generating episodes took: {round(time.time() - now, 3)}[s], steps: {time_steps}")
                    planned_vehicles = self.parse_results(episode)
                    if episode is not None and len(episode) != 0:
                        episodes += episode
                else:  # Clear previous
                    time_steps = 0
                    planned_vehicles = []
                # Continue to next step
                counter += 1
                # if counter > 100:
                #     break
        print(total)
        # Save vehicles in new scenario
        extractor = VehicleExtractor(self.scenario.vehicles_file, self.scenario.routes_file)
        entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
        for vehicle in entry.vehicles.values():
            vehicle = vehicle.to_xml()
            route: Element = deepcopy(entry.original_routes[vehicle.attrib["route"]])
            if vehicle.attrib["id"] in self.scheduler.queue.vehicles:
                route.attrib["edges"] = " ".join(self.scheduler.queue.vehicles[vehicle.attrib["id"]].route)
            route_id: str = self.new_scenario.routes_file.add_route(route, re_index=True)
            vehicle.attrib["route"] = route_id
            self.new_scenario.vehicles_file.add_vehicle(vehicle)
        return episodes

    # -------------------------------------------- Simulation --------------------------------------------

    def generate_episode(self, scheduled: List[SumoVehicle], simulation: Simulation) -> Optional[List[PddlEpisode]]:
        """
        :param scheduled: Vehicles scheduled for planning
        :param simulation: Current running SUMO simulation
        :return: List of generated PddlEpisodes (each region has its own problem & result pair)
        """
        assert (simulation is not None and simulation.is_running(use_end_time=False))
        if scheduled is None or not scheduled:
            return None
        # Generate PddlProblems for scheduled vehicles
        problems: List[PddlProblem] = self.generate_problems(scheduled, simulation)
        if problems is None or not problems:
            return None
        # Run planner for generated problems (at most len(regions) at once)
        assert (len(problems) <= 3)
        results: List[Optional[PddlResult]] = self.result_generator.generate_results(
            problems, self.options.planning.domain,
            self.options.planning.planner,
            self.new_scenario.scenario_dir,
            self.timeout,
            len(problems)
        )
        if results is None or not results:
            print("Error while generating pddl results!")
            return None
        return [PddlEpisode(i, problem, result) for i, (problem, result) in enumerate(zip(problems, results))]

    def generate_problems(self, scheduled: List[SumoVehicle], simulation: Simulation) -> List[Optional[PddlProblem]]:
        """
        :return:
        """
        assert(simulation is not None and simulation.is_running(use_end_time=False))
        interval: Tuple[int, int] = (int(simulation.get_time() - self.window), int(simulation.get_time()))
        if not scheduled:
            print(f"No vehicles scheduled in interval: {interval}")
            return []
        # Split vehicles based on their regions, create PddlProblem for each
        entries: List[VehicleEntry] = [VehicleEntry(interval) for _ in range(len(self.sub_graphs))]
        for sumo_vehicle in scheduled:
            segment: SumoRoute = sumo_vehicle.get_current_segment()
            route_id: str = sumo_vehicle.get_attribute("route")
            region_id: int = segment.region_id
            entries[region_id].vehicles[sumo_vehicle.id] = deepcopy(sumo_vehicle)
            entries[region_id].original_routes[route_id] = Element("route", {"id": route_id, "edges": " ".join(segment.region_segment)})
        problems: List[Optional[PddlProblem]] = []
        # Generate problem for each non-empty vehicle entry
        for region_id, entry in enumerate(entries):
            if not entry.vehicles:
                continue
            problems.append(
                self.problem_generator[region_id].generate_problem(
                    entry, f"problem_{interval[0]}_{interval[1]}_{self.sub_graph_names[region_id]}",
                    self.options.planning.domain
                )
            )
        return [problem for problem in problems if problem is not None and problem.is_valid()]

    # ------------------------------- Utils -------------------------------

    def parse_results(self, episodes: List[PddlEpisode]) -> List[Dict[Element, Element]]:
        """
        :param episodes:
        :return:
        """
        if not episodes:
            return []
        ret_val: List[Dict[Element, Element]] = []
        for episode in episodes:
            region_id: int = -1
            for i, region_name in enumerate(self.sub_graph_names):
                if episode.problem.name.endswith(region_name):
                    region_id = i
                    break
            assert(region_id != -1)
            parsed: Optional[Dict[Element, Element]] = self.parser[region_id].process_result(episode)
            if parsed is None or not parsed:
                print(f"Unable to parse problem: {episode.problem.name}")
                continue
            ret_val.append(parsed)
            episode.free_mem()
        return ret_val

    def assign_routes(self, planned_vehicles: List[Dict[Element, Element]], allowed: Set[str]) -> bool:
        """
        :param planned_vehicles: to which route is assigned
        :param allowed: route
        :return: True on success, False otherwise
        """
        if not allowed or not planned_vehicles:
            return True
        # Pre-process
        all_vehicles: List[Tuple[Element, Element]] = []
        sumo_vehicles: List[SumoVehicle] = []
        for entry in planned_vehicles:
            for vehicle, route in entry.items():
                if vehicle.attrib["id"] in allowed:
                    all_vehicles.append((vehicle, route))
                    sumo_vehicles.append(self.scheduler.queue.vehicles[vehicle.attrib["id"]])
                    # assert(vehicle.id in self.scheduler)
        assert(len(all_vehicles) == len(sumo_vehicles))
        # Assign vehicles
        segment_index: int = 0
        for sumo_vehicle, (vehicle, route) in zip(sumo_vehicles, all_vehicles):
            # print(f"Assigning route to vehicle: {sumo_vehicle.id}")
            if sumo_vehicle.id in self.scheduler.queue.running:
                assert((sumo_vehicle.index - 1) >= 0)
                segment_index = sumo_vehicle.index - 1
            else:
                segment_index = sumo_vehicle.index
            segment: SumoRoute = sumo_vehicle.segments[segment_index]
            edges: List[str] = route.attrib["edges"].split()
            current: int = traci.vehicle.getRouteIndex(sumo_vehicle.id)
            if (segment.region_segment[0] != edges[0]):
                print(f"Error, segment[0] != edges[0] -> {segment.region_segment[0], edges[0]}")
                continue
            assert(segment.region_segment[-1] == edges[-1])
            assert(current < segment.first_edge_index)
            # TODO check if vehicle is on internal lane
            end: int = ((segment.first_edge_index-1) + len(segment.region_segment))
            new_route: List[str] = deepcopy(sumo_vehicle.route)
            assert(new_route[(segment.first_edge_index-1):end] == segment.region_segment)
            size_diff: int = len(edges) - len(segment.region_segment)
            new_route[(segment.first_edge_index-1):end] = edges
            try:
                traci.vehicle.setRoute(sumo_vehicle.id, new_route[current:])
                sumo_vehicle.route = new_route
                segment.planned_segment = edges
                for sumo_segment in sumo_vehicle.segments[(segment_index + 1):]:
                    # Change entry to network, if the segments are connected
                    if sumo_segment.region_segment[0] == segment.region_segment[-2]:
                        sumo_segment.region_segment[0] = edges[-2]
                    sumo_segment.first_edge_index += size_diff
                    end = ((sumo_segment.first_edge_index - 1) + len(sumo_segment.region_segment))
                    assert(sumo_vehicle.route[(sumo_segment.first_edge_index - 1):end] == sumo_segment.region_segment)
            except traci.exceptions.TraCIException as e:
                print(f"Error when assigning route to vehicle: {sumo_vehicle.id}")
                continue
        return True

    def save_result(self, episode: PddlEpisode, free_mem: bool = True) -> bool:
        """
        :param episode: to be saved (i.e. vehicles and their new routes)
        :param free_mem: True if memory of episode should be freed (network, vehicles, etc.)
        :return: True on success, false otherwise
        """
        # Check episode
        if episode is None or episode.problem is None:
            print("Error, received invalid episode!")
            return False
        if free_mem:
            episode.free_mem()
        return True

    def get_steps(self, total_time: float, step_length: float) -> int:
        """
        :param total_time: time passed
        :param step_length: time of SUMO simulation step
        :return: Number of simulation steps needed for 'total_time' to pass (minimum 1).
        """
        counter: int = 0
        while total_time > 0:
            total_time -= step_length
            counter += 1
        return max(counter, 1)
