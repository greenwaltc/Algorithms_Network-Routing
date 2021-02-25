#!/usr/bin/python3
import math

from CS312Graph import *
import time
from typing import Final  # For declaring constants

DIS = 'distance'
PREV = 'previous'

class NetworkRoutingSolver:
    def __init__(self):
        pass

    def initializeNetwork(self, network):
        assert (type(network) == CS312Graph)
        self.network = network

    def getShortestPath(self, destIndex):
        self.dest = destIndex
        path_edges = []
        total_length = 0
        index = self.dest
        while index != self.source:
            latter = self.network.nodes[index]
            index_for_prev = self.nodes[index][PREV]
            if index_for_prev is None:  # Node is unreachable
                return {'cost': float('inf'), 'path': []}
            former = self.network.nodes[index_for_prev]

            length = 0
            for i in range(3):
                if former.neighbors[i].dest.node_id == latter.node_id:
                    length = former.neighbors[i].length
                    total_length += length
                    break
            edge = CS312GraphEdge(latter, former, length)
            path_edges.append((edge.src.loc, edge.dest.loc, '{:.0f}'.format(edge.length)))
            index = self.nodes[index][PREV]

        return {'cost': total_length, 'path': path_edges}

    def computeShortestPaths(self, srcIndex, use_heap=False):
        self.source = srcIndex
        t1 = time.time()

        # Initialize array with node distances and previous indexes
        self.nodes = [{
            DIS: float('inf'),
            PREV: None
        } for i in range(len(self.network.nodes))]

        # Set distance of srcIndex to 0 because that's our start node
        self.nodes[srcIndex][DIS] = 0

        if not use_heap:
            queue = self.makeQueue_array(self.nodes)
            queueEmpty = False

            while not queueEmpty:
                u, queueEmpty = self.deleteMin_array(queue)
                assert (type(u) == CS312GraphNode)
                for edge in u.neighbors:
                    if self.nodes[edge.dest.node_id][DIS] > self.nodes[edge.src.node_id][DIS] + edge.length:  # if dist(v) > dist(u) + l(u,v)
                        self.nodes[edge.dest.node_id][DIS] = self.nodes[edge.src.node_id][DIS] + edge.length  # dist(v) = dist(u) + l(u,v)
                        self.nodes[edge.dest.node_id][PREV] = u.node_id  # prev(v) = u
                        self.decreaseKey_array(queue, edge.dest)  # decreaseKey(H,v)

        else:  # Binary heap priority queue implementation
            queue, pointer_array = self.makeQueue_heap(self.nodes)

            while len(queue) > 1:
                u = self.deleteMin_heap(queue, pointer_array)
                assert (type(u) == CS312GraphNode)
                for edge in u.neighbors:
                    if self.nodes[edge.dest.node_id][DIS] > self.nodes[edge.src.node_id][DIS] + edge.length:  # if dist(v) > dist(u) + l(u,v)
                        self.nodes[edge.dest.node_id][DIS] = self.nodes[edge.src.node_id][DIS] + edge.length  # dist(v) = dist(u) + l(u,v)
                        self.nodes[edge.dest.node_id][PREV] = u.node_id  # prev(v) = u
                        self.decreaseKey_heap(queue, pointer_array, edge.dest)  # decreaseKey(H,v)

        t2 = time.time()
        return (t2 - t1)

    def makeQueue_array(self, nodes):
        queue = []
        for i in range(len(nodes)):
            queue.append(nodes[i][DIS])
        return queue

    def makeQueue_heap(self, nodes):
        queue = [{
            'node_id': None,
            'distance': None
        } for i in range(len(nodes) + 1)]
        # Initialize the array to correct size, but with nil in every position
        # At each index, queue has format {'node_id':___, 'distance':___}

        pointer_array = [-1 for i in range(len(nodes))]
        # Index in pointer array is node_id, value is index into binary heap array

        # Add element to the array using the insert function
        for i in range(len(nodes)):
            self.insert(queue, pointer_array, i, nodes[i][DIS], i + 1)

        return queue, pointer_array  # + 1 because index 0 doesn't actually have a node

    def deleteMin_array(self, queue, isEmpty=True):
        minIndex = 0

        # Find the first element of queue that is not 'none'
        while minIndex != len(queue) and queue[minIndex] is None:
            minIndex += 1

        if queue[0] == 0:
            isEmpty = False
            queue[0] = None
            node = self.network.nodes[0]
            return node, isEmpty

        # Find element with smallest distance
        for i in range(len(queue)):
            if queue[i] is not None and \
                    queue[i] < queue[minIndex]:
                minIndex = i
                isEmpty = False

        # Find the node with nodeID just removed from queue
        queue[minIndex] = None
        node = self.network.nodes[minIndex]
        return node, isEmpty

    def deleteMin_heap(self, queue, pointer_array):

        if len(queue) == 2:
            return self.network.nodes[queue.pop(1)['node_id']]
            # This is the last element in the queue, so just pop it off

        # Get the node at the top of the heap
        min_node = self.network.nodes[queue[1]['node_id']]
        in_order_predecessor = queue.pop(len(queue) - 1)

        # Reassign front of priority queue to the predecessor
        queue[1] = in_order_predecessor

        # Update indexes in pointer array
        pointer_array[min_node.node_id] = -1  # Since it is no longer in the heap
        pointer_array[in_order_predecessor['node_id']] = 1

        # Trickle down until new root is in the correct position
        self.trickle_down(queue, pointer_array)
        return min_node

    def decreaseKey_array(self, queue, v):
        assert (type(v) == CS312GraphNode)
        queue[v.node_id] = self.nodes[v.node_id][DIS]

    def decreaseKey_heap(self, queue, pointer_array, v):
        distance = self.nodes[v.node_id][DIS]
        node_index = pointer_array[v.node_id]
        queue[node_index]['distance'] = distance
        self.bubble_up(queue, pointer_array, v.node_id, node_index)


    # HEAP PRIORITY QUEUE FUNCTIONS
    def insert(self, queue, pointer_array, node_id, distance, insert_index):
        return_index = insert_index

        to_add = {
            'node_id': node_id,
            'distance': distance
        }

        pointer_array[node_id] = insert_index
        queue[insert_index] = to_add

        # While the distance of the just added node is less than its parent, bubble up.
        self.bubble_up(queue, pointer_array, node_id, insert_index)
        return return_index

    def bubble_up(self, queue, pointer_array, node_id, index):
        while math.floor(index/2) != 0 and \
                queue[index]['distance'] < queue[math.floor(index/2)]['distance']:
            # Swap the two positions
            queue[index], queue[math.floor(index/2)] = queue[math.floor(index/2)], queue[index]

            # Update indexes in pointer array
            pointer_array[node_id] = math.floor(index/2)
            pointer_array[queue[index]['node_id']] = index
            index = math.floor(index/2)

    def trickle_down(self, queue, pointer_array):

        # We will only ever have to trickle down starting at the root
        node_index = 1
        node = queue[node_index]
        left_child_index = node_index * 2
        right_child_index = (node_index * 2) + 1

        while True:
            # Get left child node
            left_child = None
            if left_child_index < len(queue):
                left_child = queue[left_child_index]

            # Get right child node
            right_child = None
            if right_child_index < len(queue):
                right_child = queue[right_child_index]

            # Run through different cases of children
            # Case 1: There are no children, so the node is already in the correct spot
            if right_child is None and left_child is None:
                break

            # Case 2: The node has both children, swap with the minimum of the two, unless the node is already the
            # min, in which case it's in the right spot
            if left_child is not None and right_child is not None:

                # Sub-case: node is less than or equal to both children, so break the loop
                if node['distance'] <= left_child['distance'] and node['distance'] <= right_child['distance']:
                    break

                # Swap the node with the smaller of the two children, or left child if they're equal
                if left_child['distance'] == right_child['distance'] or \
                        left_child['distance'] < right_child['distance']:
                    self.swap_child(queue, pointer_array, node_index, left_child_index)
                    # update node index
                    node_index = left_child_index

                else:
                    self.swap_child(queue, pointer_array, node_index, right_child_index)
                    # update node index
                    node_index = right_child_index

            # Case 3: Heap will never have right child and no left child

            # Case 4: node has left child and the child is less than the node
            elif right_child is None and left_child['distance'] < node['distance']:
                self.swap_child(queue, pointer_array, node_index, left_child_index)
                # update node index
                node_index = left_child_index

            else:
                break

            # Update children indexes for next loop through
            left_child_index = node_index * 2
            right_child_index = (node_index * 2) + 1

    def swap_child(self, queue, pointer_array, parent, child):
        # swap with the child
        queue[child], queue[parent] = queue[parent], queue[child]
        # update pointer array
        pointer_array[queue[parent]['node_id']] = parent
        pointer_array[queue[child]['node_id']] = child
