from utc.src.constants.static.file_constants import FileExtension
from utc.src.constants.file_system.my_file import MyFile
from utc.src.routing.pddl.info.episode_info import ResultInfo
from typing import Dict, List


class PddlResult:
    """
    Class representing pddl result files
    """
    def __init__(self, name: str, files: List[str]):
        """
        :param name: of pddl result file
        :param files: of this pddl result instance (can be multiple), full path
        :raise ValueError: if files are empty
        """
        self.name: str = name
        self.files: List[str] = files
        # Checks
        if not files:
            raise ValueError(f"Error, received empty list of files for pddl result: '{self.name}'")
        # Make sure the pddl extension is last and the files are correct
        for index, file in enumerate(files):
            assert(MyFile.file_exists(file))
            assert(MyFile.get_file_name(file).startswith(self.name))
            if not file.endswith(FileExtension.PDDL):
                files[index] = file.replace(FileExtension.PDDL, "") + FileExtension.PDDL
                assert(MyFile.rename_file(file, files[index]))
        self.info: ResultInfo = ResultInfo(self.name)

    def parse_result(self) -> Dict[str, List[int]]:
        """
        Parses result files of this pddl result file, replaces results (car and route pairs) in case of multiple
        being generated for the same problem file (assumes lexicographical ordering).

        :return: Dictionary mapping vehicle id (abstract) to list of route id's (internal)
        :raise ValueError: if files are empty
        """
        # Checks
        if not self.files:
            raise ValueError(f"Error, empty list of files in pddl result: '{self.name}'")
        paths: Dict[str, List[int]] = {}
        # Replace previous pddl result by next (assuming lexicographical ordering for better results)
        car_id, route_id = "", 0
        for file in self.files:
            curr_paths: Dict[str, List[int]] = {}
            with open(file, "r") as pddl_result:
                for line in pddl_result:
                    line = line.rstrip()
                    assert(line.startswith("(") and line.endswith(")"))
                    line = line[1:-1].split()
                    # MIP
                    if line[0].startswith("v"):
                        assert (line[2].startswith("r"))
                        car_id = line[0]
                        route_id = int(line[2][1:])
                    # Mercury
                    else:
                        assert(line[1].startswith("v"))
                        assert (line[3].startswith("r"))
                        car_id = line[1]
                        route_id = int(line[3][1:])
                    if car_id not in curr_paths:
                        curr_paths[car_id] = []
                    curr_paths[car_id].append(route_id)
            # Replaces keys by new ones
            paths |= curr_paths
        return paths
