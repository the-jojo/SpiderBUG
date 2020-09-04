import math
import time
from copy import deepcopy

import networkx as nx
import numpy as np

from src.nav.ObstacleSegment import ObstacleSegment
from src.geom.Cone import Cone
from src.geom.Node import Node
from src.utils.DubinsPath import find_path_weight
from src.utils.sbMath import is_point_behind_ref


class DynamicWeb:
    """
    MTG's dynamic web that spans 3D space-time, connecting obstacle tangent points
    """

    def __init__(self, cur_pos: Node, goal_pos: Node, cur_heading: float, robot_speed, logger):
        """initialise web"""
        self.logger = logger
        self.best_paths = []

        self.cur_pos = cur_pos
        self.cur_heading = cur_heading
        self.goal_pos = goal_pos
        self.goal_pos.z(self.cur_pos.z() + self.goal_pos.dist_2d(self.cur_pos) / robot_speed)

        self.cur_path = [self.cur_pos, self.goal_pos]
        self.best_paths.append(self.cur_path)

        self.DG = nx.DiGraph()
        self.DG.add_edge(self.cur_pos, self.goal_pos, weight=self.cur_pos.dist_2d(self.goal_pos))

    def move_rob_to(self, new_pos: Node, new_heading: float):
        """
        updates robot's current position in web, removing the old node
        :param new_pos: new position node
        :param new_heading: new heading
        """
        self.DG.add_node(new_pos)
        if self.cur_pos is not None and not self.cur_pos == new_pos:
            self.DG.remove_node(self.cur_pos)
        self.cur_pos = new_pos
        self.cur_heading = new_heading

    def update_shortest_path(self, turn_radius, is_sphere=False):
        """
        Dijkstra's algorithm to find the shortest path through the web
        :param turn_radius: robot's turn radius
        :param is_sphere: if the robot is holonomic
        :return: list or nodes on shortest path to goal or None if no path exists
        """
        time_0 = time.perf_counter()
        settled_nodes = []
        unsettled_nodes = [self.cur_pos]
        distances = {self.cur_pos: 0.}
        predecessors = {}

        def get_dist(_node):
            nonlocal distances
            if _node in distances:
                return distances[_node]
            return math.inf

        def get_min(_nodes):
            _min = _nodes[0]
            for _n in _nodes:
                if get_dist(_n) < get_dist(_min):
                    _min = _n
            return _min

        def get_neighbors(_node):
            nonlocal settled_nodes
            res_l = []
            for _n in nx.all_neighbors(self.DG, _node):
                if not _n in settled_nodes:
                    res_l.append(_n)
            return res_l

        def calc_distances(_node):
            nonlocal distances, predecessors, unsettled_nodes
            for _target in get_neighbors(_node):
                if _node == self.cur_pos:
                    if is_sphere:
                        d = _node.dist_2d(_target)
                    else:
                        d = find_path_weight(_node, self.cur_heading, [_target], turn_radius)
                else:
                    d = _node.dist_2d(_target)
                if get_dist(_target) > (get_dist(_node) + d):
                    distances[_target] = get_dist(_node) + d
                    predecessors[_target] = _node
                    unsettled_nodes.append(_target)

        while len(unsettled_nodes) > 0:
            n = get_min(unsettled_nodes)
            settled_nodes.append(n)
            unsettled_nodes.remove(n)
            calc_distances(n)

        if self.goal_pos in predecessors:
            step = self.goal_pos
            res_path = [self.goal_pos]
            while step in predecessors:
                step = predecessors[step]
                res_path.insert(0, step)

            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                print("Navigation update_shortest_path(): took : %d sec" % (time_1 - time_0))
            return res_path
        else:
            time_1 = time.perf_counter()
            if time_1 - time_0 > 2:
                print("Navigation update_shortest_path(): took : %d sec" % (time_1 - time_0))

            return None

    def rem_near_nodes_from_path(self, cur_pos: Node, removal_dist):
        """
        Removes nodes behind the robot's current position as they are irrelevant
        :param cur_pos: robot's current position
        :param removal_dist: threshold used to determine if nodes are sufficiently behind position
        """
        if self.cur_path is not None and len(self.cur_path) > 0:
            p_p: [Node] = [Node.from_list(x.as_list_3d()) for x in self.cur_path]
            for i in np.arange(1, len(p_p)):
                if cur_pos.dist_2d(p_p[i]) < removal_dist:
                    self.cur_path.remove(p_p[i])

    def update(self, obstacles: [ObstacleSegment], new_rob_pos: Node, robot_speed: float, tangent_dist_out: float,
               tangent_dist_in: float, removal_dist: float, rob_radius: float):
        """
        Web update routine.
         - removes points behind robot
         - add new tangent points
         - prunes old nodes if web becomes too complicated
         - interconnect all nodes using sensed obstacleSegments
        """
        # remove all edges
        self.DG = nx.create_empty_copy(self.DG)

        # remove points behind robot except start
        nodes = deepcopy(self.DG.nodes)
        for node in nodes:
            if is_point_behind_ref(node.as_list_2d(), self.cur_pos.as_list_2d(), self.goal_pos.as_list_2d(), removal_dist):
                self.DG.remove_node(node)

        # add new tangent points
        rob_cone = Cone(new_rob_pos, robot_speed)
        n_to_add = []
        for obst in obstacles:
            tangent_points = obst.get_tangent_points_3d(rob_cone, tangent_dist_out, tangent_dist_in, self.logger)
            if tangent_points is None or len(tangent_points) == 0:
                pass
            else:
                n_to_add.extend(tangent_points)

        if (len(n_to_add) + len(self.DG.nodes)) > (len(obstacles) * 3 + 4):
            self.logger.debug("Navigation update(): Pruning web to avoid performance penalties")
            for n in deepcopy(self.DG.nodes):
                if not n == self.cur_pos and not n == self.goal_pos:
                    self.DG.remove_node(n)

        self.add_nodes_2d(n_to_add, removal_dist)

        # interconnect all points
        self.interconnect_from(new_rob_pos, robot_speed, obstacles, rob_radius, tangent_dist_in, removal_dist)

    def add_nodes_2d(self, new_nodes: [Node], removal_dist):
        """Adds new nodes to graph but removes all old nodes that were within removal_dist of new ones"""
        for new_node in new_nodes:
            # remove old nodes within removalDist
            nodes = deepcopy(self.DG.nodes)
            for node in nodes:
                # do not remove goal or start points
                if not (node == self.cur_pos or node == self.goal_pos) and \
                        node.dist_2d(new_node) <= removal_dist:
                    self.DG.remove_node(node)

            self.DG.add_node(new_node.as_node_2d())

    def get_nodes_by_dist(self, ref_point: Node) -> [Node]:
        """Returns nodes in increasing order of distance to ref point"""
        nodes = np.array([(ref_point.dist_2d(x), x) for x in self.DG.nodes])
        return nodes[np.argsort(nodes[:, 0])][:, 1]

    def interconnect_from(self, new_pos_3d: Node, robot_speed: float, obstacles: [ObstacleSegment],
                          rob_radius: float, tangent_dist_in: float, removal_dist: float):
        """
        Interconnects nodes of web, starting with the robot's current position.
        Updates nodes' 3D position along the way.
        Procedure:
         - For each node b in other nodes sorted in ascending order by distance to current position:
            - if a clear path exists to node b:
                - calculate node b's z coordinate by using distance over speed
                - add edge to node b
                - add node b to processed nodes
         - while there are nodes in the processed nodes:
            - for each node a in processed nodes:
                - for each node b in other nodes sorted in ascending order by distance to node a:
                    - if clear path to node b exists:
                        - if we found a shorter path to node b since the new z coordinate is less than the old one:
                            - update node b's z coordinate
                            - remove all previous out-edges of node-b
                            - add edge node a to node b
                            - add node b to processed nodes
                        - else:
                            - add edge node a to node b
        """
        time_0 = time.perf_counter()
        self.DG.add_node(new_pos_3d)
        nodes_i = deepcopy(self.get_nodes_by_dist(new_pos_3d))
        processed_nodes: [Node] = []
        # loop all nearby nodes
        for node_i in nodes_i[1:]:
            # check if the path from newPos to node_i is clear
            t_d = tangent_dist_in
            if not DynamicWeb.__intersects__(obstacles, [node_i.as_node_2d()], new_pos_3d, robot_speed,
                                             t_d):
                # remove 2d node
                self.DG.remove_node(node_i)
                # add 3d node
                z = new_pos_3d.z() + (new_pos_3d.dist_2d(node_i) / robot_speed)
                node_i_3d = Node.from_list([node_i.x(), node_i.y(), z])
                self.DG.add_node(node_i_3d)
                # add edge to 3d node
                self.DG.add_edge(new_pos_3d, node_i_3d)
                # add to list
                processed_nodes.append(node_i_3d)

                if self.goal_pos.dist_2d(node_i) < removal_dist:
                    self.goal_pos = node_i_3d
        # now we have edges to all immediately reachable nodes and updated them to 3d
        # loop all directly reachable and create edges from there
        while len(processed_nodes) > 0:
            for dr_node in processed_nodes:
                processed_nodes.remove(dr_node)
                # dr_node = Node(dr_node)
                # loop all nearby nodes
                nodes = deepcopy(self.get_nodes_by_dist(dr_node))
                for node in nodes:
                    # dont create edges to newPos (we already have) nor themselves nor to blocked nodes
                    if (not node == dr_node) and (not node == new_pos_3d) and \
                            not DynamicWeb.__intersects__(obstacles, [node.as_node_2d()], dr_node, robot_speed,
                                                     1.01*rob_radius):
                        # find 3d location
                        z = dr_node.z() + (dr_node.dist_2d(node) / robot_speed)
                        # compare new z to old z
                        if z < node.z():
                            # found path to node that was quicker than previous best path
                            # preserve other edges to node
                            prev_edges_in = deepcopy(self.DG.in_edges(node))

                            # remove old node
                            self.DG.remove_node(node)
                            new_node = Node.from_list([node.x(), node.y(), z])
                            # add new node and edge to node
                            self.DG.add_node(new_node)
                            self.DG.add_edge(dr_node, new_node)
                            # add back in edges
                            for e_in in prev_edges_in:
                                self.DG.add_edge(e_in[0], new_node)

                            # add to processed_nodes
                            processed_nodes.append(new_node)

                            if self.goal_pos.dist_2d(node) < removal_dist:
                                self.goal_pos = new_node
                        else:
                            # new path is not shorter
                            # still create edge
                            self.DG.add_edge(dr_node, node)
        time_1 = time.perf_counter()
        if time_1 - time_0 > 2:
            print("Navigation interconnect(): took : %d sec" % (time_1 - time_0))

    def reset_paths(self):
        self.cur_path = None
        self.best_paths = []

    @staticmethod
    def __intersects__(obstacles: [ObstacleSegment], path_2d: [Node], start_pos_3d: Node, rob_speed: float,
                       tangent_dist_in: float, direct_only = False) -> bool:
        """Returns True if path intersects obstacles; False otherwise"""
        for obst in obstacles:
            v1 = obst.get_intersect_with_path_3d(start_pos_3d, path_2d, rob_speed, tangent_dist_in, direct_only)
            if v1 is not None:
                return True
        return False
