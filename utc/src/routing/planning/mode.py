from utc.src.routing.pddl.pddl_episode import PddlEpisode
from utc.src.routing.pddl.pddl_options import PddlOptions
from utc.src.routing.traffic import ResultGenerator, ProblemGenerator, Parser
from utc.src.simulator.scenario import Scenario
from utc.src.graph import Graph, RoadNetwork
from typing import Optional, Union, List


class Mode:
    """
    Super Class for online & offline type planning,
    provides utility parameters and acts as an interface
    """
    def __init__(self, options: PddlOptions):
        """
        :param options: of Pddl generation
        """
        self.options: PddlOptions = options
        self.scenario: Optional[Scenario] = None
        self.graph: Optional[Graph] = None
        self.new_scenario: Optional[Scenario] = None
        self.problem_generator: Optional[Union[ProblemGenerator, List[ProblemGenerator]]] = None
        self.result_generator: Optional[ResultGenerator] = None
        self.parser: Optional[Union[Parser, List[Parser]]] = None
        assert(self.initialize())
        print(f"Successfully initialized PDDL {self.__class__.__name__} mode.")

    def initialize(self) -> bool:
        """
        :return: True on success, false otherwise.
        """
        raise NotImplementedError("Error, method 'initialize' must be implemented by children of 'Mode' class!")

    def generate_episodes(self) -> Optional[List[PddlEpisode]]:
        """
        Class generating pddl episodes (problems & results).

        :return: List of pddl episodes generated, None if error occurred
        """
        raise NotImplementedError("Error, method 'generate_episodes' must be implemented by children of 'Mode' class!")

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
        for (vehicle, route) in self.parser.process_result(episode).items():
            route_id: str = self.new_scenario.routes_file.add_route(route, re_index=True)
            vehicle.attrib["route"] = route_id
            self.new_scenario.vehicles_file.add_vehicle(vehicle)
        if free_mem:
            episode.free_mem()
        return True













