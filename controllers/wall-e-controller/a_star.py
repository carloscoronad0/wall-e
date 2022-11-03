import math
from enum import Enum
from dataclasses import dataclass
from typing import Union
from numpy._typing import NDArray

@dataclass
class GridNode:
    position: tuple[int,int]
    previous: Union[tuple[int,int], None]
    g_score: float = 1000000.
    h_score: float = 1000000.
    f_score: float = 1000000.

    def x_element(self):
        return self.position[0]

    def y_element(self):
        return self.position[1]

    def calculate_f_score(self):
        self.f_score = self.g_score + self.h_score
        return self.f_score

class SearchState(Enum):
        STILL_SEARCHING = 1
        GOAL_NOT_POSSIBLE = 2
        GOAL_REACHED = 3

class AStar:
    def __init__(self) -> None:
        self.unvisited_dict: dict[tuple[int,int], GridNode] = {}
        self.visited_dict: dict[tuple[int,int], GridNode] = {}

    @staticmethod
    def euclidean_distance(node_pos_1: tuple[int, int], node_pos_2: tuple[int,int]) -> float:
        a = pow(node_pos_1[0] - node_pos_2[0], 2)
        b = pow(node_pos_1[1] - node_pos_2[1], 2)
        return math.sqrt(a + b)

    @staticmethod
    def get_node_neighbors_pos(node_pos: tuple[int, int], grid: NDArray, filter_reference: int = 0):
        neighbors = []
        x_pos, y_pos = node_pos
        map_width, map_long = grid.shape

        neighbors.extend([(x_pos + step, y_pos)
                          for step in [-1,1]
                          if (x_pos + step >= 0) 
                          and (x_pos + step < map_width)
                          and (grid[x_pos + step][y_pos] >= filter_reference)])

        neighbors.extend([(x_pos, y_pos + step)
                          for step in [-1,1]
                          if (y_pos + step >= 0) 
                          and (y_pos + step < map_long)
                          and (grid[x_pos][y_pos + 1] >= filter_reference)])

        return neighbors

    def __insert_into_unvisited__(self, new_node_pos, start, goal, previous_node_pos = None):
        self.unvisited_dict[new_node_pos] = GridNode(
                position=new_node_pos,
                previous=previous_node_pos
                )

        if previous_node_pos:
            self.unvisited_dict[new_node_pos].g_score = self.euclidean_distance(start, new_node_pos)
            self.unvisited_dict[new_node_pos].h_score = self.euclidean_distance(new_node_pos, goal)
            self.unvisited_dict[new_node_pos].calculate_f_score()

    def __mark_as_visited__(self, node_pos):
        self.visited_dict[node_pos] = self.unvisited_dict.pop(node_pos)

    def __initialize_neighbors_on_unvisited__(
            self,
            node_position: tuple[int,int],
            neighbors: list[tuple[int,int]],
            start,
            goal
        ):
        registered_neighbors = self.unvisited_dict.keys()
        visited_neighbors = self.visited_dict.keys()
        for neighbor_pos in neighbors:
            if (neighbor_pos not in registered_neighbors) and (neighbor_pos not in visited_neighbors):
                self.__insert_into_unvisited__(neighbor_pos, start, goal, previous_node_pos=node_position)

    def visit_neighbors(
            self,
            node_position: tuple[int, int],
            grid: NDArray,
            filter_reference: int,
            start: tuple[int,int],
            goal: tuple[int,int]
        ):
        neighbors = self.get_node_neighbors_pos(node_position, grid, filter_reference)
        self.__initialize_neighbors_on_unvisited__(node_position, neighbors, start, goal)
        self.__mark_as_visited__(node_position)

        if goal in neighbors:
            return SearchState.GOAL_REACHED

        return SearchState.STILL_SEARCHING

    def get_lowest_f_score_from_unvisited(self):
        sorted_unvisited = sorted(self.unvisited_dict.items(), key=lambda x: x[1].f_score)
        return sorted_unvisited[0][0]

    def perform_step(self, start: tuple[int,int], goal: tuple[int,int], grid: NDArray):
        lowest = self.get_lowest_f_score_from_unvisited()
        search_state = self.visit_neighbors(lowest, grid, 0, start, goal)

        if (search_state == SearchState.STILL_SEARCHING) and (not self.unvisited_dict):
            return SearchState.GOAL_NOT_POSSIBLE

        return search_state

    def search_for_optimal_path(self, start: tuple[int,int], goal: tuple[int,int], grid: NDArray):
        def get_optimal_path():
            previous = self.unvisited_dict[goal].previous
            points = [goal]
            while previous != None:
                points.append(previous)
                previous = self.visited_dict[previous].previous

            return points

        self.__insert_into_unvisited__(start, start=start, goal=goal)
        step = self.perform_step(start, goal, grid)
        while step == SearchState.STILL_SEARCHING:
            step = self.perform_step(start, goal, grid)

        if step == SearchState.GOAL_NOT_POSSIBLE:
            return []
        else:
            return get_optimal_path()
