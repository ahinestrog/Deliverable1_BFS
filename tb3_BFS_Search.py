import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from collections import deque
import time

class BFSSearch(Node):
    def __init__(self):
        super().__init__('bfs_search')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.execute_path)
        
        # Definir secuencia de movimientos basada en BFS
        path = self.bfs_algorithm()
        new_path = []
        # traducir las acciones a comandos coherentes para el robot
        self.moves = deque(path) # Convierte el path una cola doble
        self.current_move = None
        self.start_time = None

    def bfs_algorithm(self):
        # Método que el estudiante debe completar para calcular el path solución
        # Retorna un array de comandos de dirección 'forward', 'left', ...
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

        start = (3,1)
        end = (1, 12)

        actions = {
            "up": (-1, 0),
            "down": (1, 0),
            "right":(0, 1),
            "left": (0, -1)
        }
        queue = deque([start]) # Cola donde vamos a almacenar todos los caminos posibles
        reached = {start : None}

        while queue:
            x, y = queue.popleft() # Saca la posición primera en la cola
            print(f"Visitando: {(x, y)}")
            if (x, y) == end: # Verifica que ya estemos en el final
                print("¡Meta alcanzada!")
                return self.reconstruct_path(start, end, reached)
            
            for direction, (dx, dy) in actions.items():
                nx, ny = (x + dx, y + dy)

                if 0 <= nx < len(maze) and 0 <= ny < len(maze[0]) and maze[nx][ny] in [" ", "E"] and (nx, ny) not in reached:
                    queue.append((nx, ny))
                    reached[(nx, ny)] = (x, y, direction)
        return [] # No encontro camino/salida
        
    
    def reconstruct_path(self, start, end, reached):
        path = []
        current = end # Partimos cuando ya hemos llegado al final, y empezamos a reconstruir el camino hacia atrás

        while current != start: # Sí el actual es igual al inicio, ya reconstruimos todo el camino
            prev, action = reached[current][0:2], reached[current][2]  # Toma el x, y, y ya en action la dirección
            path.append(action)
            current = prev
        path.reverse()
        print("Camino reconstruido:", path)
        return path

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
        
        if action == 'up' or action == 'down':
            duration = 5.0
        elif action == 'left':
            duration = 3.15
        elif action == 'right':
            duration = 3.15
        else:
            duration = 0

        
        if action == 'up' or action == 'down':
            twist.linear.x = 0.2  # Movimiento hacia adelante
            twist.angular.z = 0.0
        elif action == 'left':
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Gira a la izquierda
        elif action == 'right':
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Gira a la derecha
        
        self.publisher_.publish(twist)
        
        while time.time() - self.start_time < duration:
            pass

        self.current_move = None  # Pasar al siguiente movimiento


def main(args=None):
    rclpy.init(args=args)
    node = BFSSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()