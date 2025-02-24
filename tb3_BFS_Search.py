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
        self.moves = deque(path)
        self.current_move = None
        self.start_time = None

    def bfs_algorithm(self):
        # Método que el estudiante debe completar para calcular el path solución
        # Retorna un array de comandos de dirección 'forward', 'left', ...
        return ['forward', 'forward']

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
            duration = 3.15
        elif action == 'turn right':
            duration = 3.15
        else:
            duration = 0

        
        if action == 'forward':
            twist.linear.x = 0.2  # Movimiento hacia adelante
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

        self.current_move = None  # Pasar al siguiente movimiento


def main(args=None):
    rclpy.init(args=args)
    node = BFSSearch()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()