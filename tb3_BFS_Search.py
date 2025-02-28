import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque
import time
import heapq
import math

class Nodo:
    def __init__(self, position, parent=None,  path_cost=0, action=None,):
        self.position = position
        self.parent = parent
        self.path_cost = path_cost
        self.action=action
        

    def __lt__(self, other):
        return self.path_cost < other.path_cost
    
class Problem:
    def __init__(self, maze, start, end,actions):
        self.maze = maze
        self.start = start
        self.end = end
        self.actions=actions

class BFSSearch(Node):
    def __init__(self):
        super().__init__('bfs_search')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.execute_path)
        
        # Definir secuencia de movimientos basada en BFS
        maze = [
            ["#", "#", "#", "#", "#", "#", "#","#", "#", "#", "#", "#", "#"],
            ["#", " ", " ", " ", " ", " ", " "," ", "#", "#", "#", " ", "E"],
            ["#", "#", "#", " ", "#", " ", "#"," ", "#", "#", "#", " ", "#"],
            ["#", "S", "#", " ", "#", " ", "#"," ", "#", "#", "#", " ", "#"],
            ["#", " ", "#", " ", "#", " ", "#","#", "#", "#", "#", " ", "#"],
            ["#", " ", "#", " ", "#", " ", " "," ", " ", " ", " ", " ", "#"],
            ["#", " ", "#", " ", "#", " ", "#"," ", "#", "#", "#", " ", "#"],
            ["#", " ", " ", " ", "#", " ", "#"," ", "#", " ", " ", " ", "#"],
            ["#", "#", "#", "#", "#", "#", "#","#", "#", "#", "#", "#", "#"]
        ]
        path = self.bfs_algorithm(maze)
        new_path = []
        # traducir las acciones a comandos coherentes para el robot
        self.moves = deque(path)
        self.current_move = None
        self.start_time = None
    
    def reconstruct_path(self, node):
        instructions=[]
        actions=[]
        while node.parent is not None:
            instructions.append(node.action)
            node = node.parent
        
        instructions.reverse()
        direction="down"
        sum_same_directions=0
        for i in instructions:
            if direction== i:
                sum_same_directions+=1
            else:
                for a in range(0,math.ceil(sum_same_directions/2)):
                    actions.append("forward")
                if direction=="down": 
                    if i=="left":
                        actions.append("turn right")
                        direction="left"
                        sum_same_directions=1
                    elif i=="right":
                        actions.append("turn left")
                        direction="right"
                        sum_same_directions=1
                    elif i=="up":
                        actions.append("turn right")
                        actions.append("turn right")
                        direction="up"
                        sum_same_directions=1
            
                elif direction=="up":
                    if i=="left":
                        actions.append("turn left")
                        direction="left"
                        sum_same_directions=1
                    elif i=="right":
                        actions.append("turn right")
                        direction="right"
                        sum_same_directions=1
                    elif i=="down":
                        actions.append("turn right")
                        actions.append("turn right")
                        direction="down"
                        sum_same_directions=1
                
                elif direction=="left":
                    if i=="up":
                        actions.append("turn right")
                        direction="up"
                        sum_same_directions=1
                    elif i=="right":
                        actions.append("turn right")
                        actions.append("turn right")
                        direction="right"
                        sum_same_directions=1
                    elif i=="down":
                        actions.append("turn left")
                        direction="dowm"
                        sum_same_directions=1
                
                elif direction=="right":
                    if i=="up":
                        actions.append("turn left")
                        direction="up"
                        sum_same_directions=1
                    elif i=="left":
                        actions.append("turn right")
                        actions.append("turn right")
                        direction="left"
                        sum_same_directions=1
                    elif i=="down":
                        actions.append("turn right")
                        direction="down"
                        sum_same_directions=1
        
        return actions

    def bfs_algorithm(self,maze):
        #Definición del problema
        start = (3, 1) 
        end = (1, 12)
        actions = {
            "up": (-1, 0),
            "down": (1, 0),
            "left": (0, -1),
            "right": (0, 1)
        }
        problem=Problem(maze, start, end,actions)

        def manhatan_distance(pos, goal):
            return abs(pos[0] - goal[0]) + abs(pos[1] - goal[1])  # Distancia de Manhattan

        def get_neighbors(pos):
            neighbors = [] #lista de vecinos
            for action, (dx, dy) in problem.actions.items():  #Tenga en cuenta que para que esto sea funcional ya debio haber definido el objeto problem
                neighbor = (pos[0] + dx, pos[1] + dy)
                if maze[neighbor[0]][neighbor[1]] != "#": #si el vecino es diferente a "#" pared agregarlo a la lista de vecinos                neighbors.append(neighbor)
                    neighbors.append((neighbor,action))
            return neighbors

        start_node = Nodo(start, path_cost=0)
        frontier = [(manhatan_distance(start, end), start_node)]
        heapq.heapify(frontier) #Convierte la lista frontier en una cola de prioridad (heap)
        reached = {start: start_node}

        while frontier:
            _, node = heapq.heappop(frontier)
            if node.position == end:
                return self.reconstruct_path(node)
            
            for neighbor, action in get_neighbors(node.position):
                new_cost = node.path_cost + 1
                if neighbor not in reached or new_cost < reached[neighbor].path_cost:
                    reached[neighbor] = Nodo(neighbor, parent=node,action=action, path_cost=new_cost)
                    heapq.heappush(frontier, (manhatan_distance(neighbor, end), reached[neighbor]))
                    
        return None

    def execute_path(self):
        twist = Twist()
        if not self.moves:
            self.get_logger().info("¡Salida encontrada! Finalizando ejecución.")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.timer.cancel()
            return
        
        if self.current_move is None:
            self.current_move = self.moves.popleft()
            self.get_logger().info(self.current_move)
            # Generar codigo para traducir las acciones o comandos coherentes para el movimiento del robot
            self.start_time = time.time()
        
        action = self.current_move
        
        if action == 'forward':
            duration = 5.0
        elif action == 'turn left':
            duration = 3.62
        elif action == 'turn right':
            duration = 3.62
        else:
            duration = 0

        
        if action == 'forward':
            twist.linear.x = 0.24 # Movimiento hacia adelante
            twist.angular.z = 0.0
        elif action == 'turn left':
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Gira a la izquierda
        elif action == 'turn right':
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Gira a la derecha
        
        self.publisher_.publish(twist)
        
        while time.time() - self.start_time < duration:
            pass

        self.current_move = None

def main(args=None):
    rclpy.init(args=args)
    node = BFSSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()