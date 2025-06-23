from utc.src.simulator.scenario import Scenario
from utc.src.simulator.simulation import Simulation
from utc.src.constants.static import DirPaths, FileExtension, FilePaths
from utc.src.utils.vehicle_extractor import VehicleExtractor, VehicleEntry
from utc.src.constants.static.pddl_constants import SOLVERS
from utc.src.constants.file_system.my_directory import MyDirectory
from utc.src.constants.file_system.file_types.xml_file import XmlFile
from utc.src.constants.file_system.file_types.sumo_config_file import SumoConfigFile
from utc.src.graph import Graph, RoadNetwork
from utc.src.utils.task_manager import TaskManager
from copy import deepcopy
from typing import Tuple, List
import traci


if __name__ == "__main__":
    scenario: Scenario = Scenario("itsc_25200_32400")
    extractor: VehicleExtractor = VehicleExtractor(scenario.vehicles_file, scenario.routes_file)
    entry: VehicleEntry = extractor.estimate_arrival_naive((0, float("inf")))
    routes: set = set()
    for vehicle in entry.vehicles.values():
        routes.add(vehicle.attributes["route"])
    print(len(routes), len(entry.original_routes))

    # v_id: str = ""
    # route: str = ""
    # found: bool = False
    #
    # with Simulation(FilePaths.SCENARIO_CONFIG.format("lust_25200_32400", "stats_routed"), {"-W": ""}) as simulation:
    #     counter: int = 0
    #     while simulation.is_running() and counter < 10000:
    #         departed = traci.simulation.getDepartedIDList()
    #         if departed and not v_id:
    #             v_id = departed[0]
    #             route = traci.vehicle.getRoute(v_id)
    #         if counter % 10 == 0 and v_id:
    #             print(traci.vehicle.getParameter(v_id, f"device.rerouting.edge:{route[0]}"))
    #             # print(traci.edge.getTraveltime(route[0]))
    #
    #         # for vehicle_id in departed
    #         #     # print("-----------------")
    #         #     route: str = ' '.join(traci.vehicle.getRoute(vehicle_id))
    #         #     route_id: str = entry.vehicles[vehicle_id].attributes["route"]
    #         #     original: str = entry.original_routes[route_id].attrib['edges']
    #         #     if route != original:
    #         #         print(f"Vehicle: {vehicle_id}, original: {original}, changed: {route}")
    #         simulation.step()
    #         counter += 1









