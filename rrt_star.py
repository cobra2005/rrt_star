import pygame
import math
import random
import sys
from typing import List, Tuple, Optional

# Màu sắc
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
LIGHT_BLUE = (173, 216, 230)

class Node:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        return hash((self.x, self.y))

class RRTStar:
    def __init__(self, 
                 start: Node, 
                 goal: Node, 
                 bounds: Tuple[float, float, float, float],
                 obstacle_list: List[Tuple[float, float, float]],
                 max_iter: int = 1000,
                 step_size: float = 20,
                 goal_sample_rate: float = 0.1,
                 gamma: float = 100.0):
        
        self.start = start
        self.goal = goal
        self.bounds = bounds
        self.obstacle_list = obstacle_list
        self.max_iter = max_iter
        self.step_size = step_size
        self.goal_sample_rate = goal_sample_rate
        self.gamma = gamma
        
        self.V = [self.start]
        self.E = []
        self.goal_nodes = []
        self.current_iter = 0
        self.found_path = None
        self.best_path = None
        
    def step(self) -> bool:
        """Thực hiện một bước của thuật toán, trả về True nếu tìm thấy đường đi"""
        if self.current_iter >= self.max_iter:
            return False
            
        x_rand = self.sample()
        x_nearest = self.nearest(x_rand)
        x_new = self.steer(x_nearest, x_rand)
        
        if self.obstacle_free(x_nearest, x_new):
            card_V = len(self.V)
            d = 2  # 2D space
            radius = min(self.gamma * (math.log(card_V) / card_V) ** (1/d), self.step_size)
            X_near = self.near(x_new, radius)
            
            self.V.append(x_new)
            
            x_min = x_nearest
            c_min = x_nearest.cost + self.line_cost(x_nearest, x_new)
            
            for x_near in X_near:
                if self.obstacle_free(x_near, x_new):
                    new_cost = x_near.cost + self.line_cost(x_near, x_new)
                    if new_cost < c_min:
                        x_min = x_near
                        c_min = new_cost
            
            x_new.parent = x_min
            x_new.cost = c_min
            self.E.append((x_min, x_new))
            
            for x_near in X_near:
                if x_near == x_min:
                    continue
                
                if self.obstacle_free(x_new, x_near):
                    new_cost = x_new.cost + self.line_cost(x_new, x_near)
                    if new_cost < x_near.cost:
                        for edge in self.E:
                            if edge[1] == x_near:
                                self.E.remove(edge)
                                break
                        
                        x_near.parent = x_new
                        x_near.cost = new_cost
                        self.E.append((x_new, x_near))
        
            # Kiểm tra nếu đến đích
            if self.distance(x_new, self.goal) < self.step_size and self.obstacle_free(x_new, self.goal):
                temp_goal = Node(self.goal.x, self.goal.y)
                temp_goal.parent = x_new
                temp_goal.cost = x_new.cost + self.line_cost(x_new, self.goal)
                self.goal_nodes.append(temp_goal)
                
                # Luôn cập nhật đường đi tốt nhất khi tìm thấy
                if not self.best_path or temp_goal.cost < self.best_path[1]:
                    path = self.extract_path(temp_goal)
                    self.best_path = (path, temp_goal.cost)
        
        self.current_iter += 1

        # Chỉ trả về True khi đã hoàn thành tất cả các lần lặp
        if self.current_iter >= self.max_iter:
            self.found_path = self.best_path
            return True
        return False

    def sample(self) -> Node:
        if random.random() < self.goal_sample_rate:
            return Node(self.goal.x, self.goal.y)
        
        x = random.uniform(self.bounds[0], self.bounds[1])
        y = random.uniform(self.bounds[2], self.bounds[3])
        return Node(x, y)
    
    def nearest(self, x_rand: Node) -> Node:
        min_dist = float('inf')
        nearest_node = None
        
        for node in self.V:
            dist = self.distance(node, x_rand)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
                
        return nearest_node
    
    def steer(self, x_nearest: Node, x_rand: Node) -> Node:
        dist = self.distance(x_nearest, x_rand)
        if dist <= self.step_size:
            return Node(x_rand.x, x_rand.y)
        
        theta = math.atan2(x_rand.y - x_nearest.y, x_rand.x - x_nearest.x)
        new_x = x_nearest.x + self.step_size * math.cos(theta)
        new_y = x_nearest.y + self.step_size * math.sin(theta)
        return Node(new_x, new_y)
    
    def obstacle_free(self, x1: Node, x2: Node) -> bool:
        for (ox, oy, radius) in self.obstacle_list:
            if self.line_circle_intersection(x1, x2, (ox, oy, radius)):
                return False
        return True
    
    def line_circle_intersection(self, x1: Node, x2: Node, circle: Tuple[float, float, float]) -> bool:
        ox, oy, r = circle
        x1x, x1y = x1.x, x1.y
        x2x, x2y = x2.x, x2.y

        dx = x2x - x1x
        dy = x2y - x1y

        fx = x1x - ox
        fy = x1y - oy

        a = dx*dx + dy*dy
        b = 2*(fx*dx + fy*dy)
        c = fx*fx + fy*fy - r*r

        if a == 0:
            return math.sqrt((x1x - ox)**2 + (x1y - oy)**2) <= r

        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return False

        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2*a)
        t2 = (-b + discriminant) / (2*a)

        if 0 <= t1 <= 1 or 0 <= t2 <= 1:
            return True

        return False
    
    def near(self, x_new: Node, radius: float) -> List[Node]:
        near_nodes = []
        for node in self.V:
            if self.distance(node, x_new) <= radius:
                near_nodes.append(node)
        return near_nodes
    
    def line_cost(self, x1: Node, x2: Node) -> float:
        return self.distance(x1, x2)
    
    def distance(self, x1: Node, x2: Node) -> float:
        return math.sqrt((x1.x - x2.x)**2 + (x1.y - x2.y)**2)
    
    def extract_path(self, goal_node: Node) -> List[Tuple[float, float]]:
        path = []
        current = goal_node
        
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
            
        return path[::-1]

def main():
    # Khởi tạo Pygame
    pygame.init()
    width, height = 1000, 800
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("RRT* Path Planning with Animation")
    clock = pygame.time.Clock()
    
    # Thiết lập môi trường
    bounds = (0, width, 0, height)
    start = Node(50, 50)
    goal = Node(width - 50, height - 50)
    
    # Tạo chướng ngại vật ngẫu nhiên
    obstacle_list = []
    for _ in range(15):
        x = random.randint(100, width - 100)
        y = random.randint(100, height - 100)
        radius = random.randint(20, 60)
        obstacle_list.append((x, y, radius))
    
    # Tạo RRT*
    rrt_star = RRTStar(
        start=start,
        goal=goal,
        bounds=bounds,
        obstacle_list=obstacle_list,
        max_iter=5000,
        step_size=25,
        goal_sample_rate=0.1,
        gamma=150.0
    )
    
    # Biến để điều khiển hiển thị
    running = True
    paused = False
    show_tree = True
    show_animation = True
    animation_speed = 1  # Số bước thực hiện mỗi frame
    best_path = None
    
    # Font cho hiển thị thông tin
    font = pygame.font.SysFont('Times New Roman', 24)
    small_font = pygame.font.SysFont('Times New Roman', 18)
    
    # Vòng lặp chính
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    paused = not paused
                elif event.key == pygame.K_t:
                    show_tree = not show_tree
                elif event.key == pygame.K_a:
                    show_animation = not show_animation
                elif event.key == pygame.K_UP:
                    animation_speed = min(animation_speed + 1, 20)
                elif event.key == pygame.K_DOWN:
                    animation_speed = max(animation_speed - 1, 1)
                elif event.key == pygame.K_r:
                    # Reset
                    main()
                    return
        
        if not paused and show_animation:
            # Thực hiện một hoặc nhiều bước của RRT*
            for _ in range(animation_speed):
                if rrt_star.step():
                    best_path = rrt_star.found_path[0] if rrt_star.found_path else None
        
        # Vẽ màn hình
        screen.fill(WHITE)
        
        # Vẽ chướng ngại vật
        for (ox, oy, radius) in obstacle_list:
            pygame.draw.circle(screen, DARK_GRAY, (int(ox), int(oy)), int(radius))
        
        # Vẽ cây (nếu được bật)
        if show_tree:
            for (node1, node2) in rrt_star.E:
                pygame.draw.line(screen, GRAY, 
                                (int(node1.x), int(node1.y)), 
                                (int(node2.x), int(node2.y)), 1)
        
        # Vẽ điểm mới nhất (để làm nổi bật animation)
        if rrt_star.V and show_animation:
            last_node = rrt_star.V[-1]
            pygame.draw.circle(screen, LIGHT_BLUE, (int(last_node.x), int(last_node.y)), 3)
            if last_node.parent:
                pygame.draw.line(screen, LIGHT_BLUE,
                               (int(last_node.parent.x), int(last_node.parent.y)),
                               (int(last_node.x), int(last_node.y)), 2)
        
        # Vẽ đường đi tốt nhất (nếu có)
        if best_path:
            for i in range(len(best_path) - 1):
                pygame.draw.line(screen, RED, 
                                (int(best_path[i][0]), int(best_path[i][1])), 
                                (int(best_path[i+1][0]), int(best_path[i+1][1])), 4)
        
        # Vẽ điểm bắt đầu và đích
        pygame.draw.circle(screen, BLUE, (int(start.x), int(start.y)), 10)
        pygame.draw.circle(screen, GREEN, (int(goal.x), int(goal.y)), 10)
        
        # Hiển thị thông tin
        info_texts = [
            f"Số lần lặp: {rrt_star.current_iter}/{rrt_star.max_iter}",
            f"Số nút: {len(rrt_star.V)}",
            f"Số đường đi tìm được: {len(rrt_star.goal_nodes)}",
            f"Tốc độ animation: {animation_speed}x (UP/DOWN để điều chỉnh)",
            f"Độ dài đường đi: {rrt_star.found_path[1]:.1f}" if rrt_star.found_path else "Đang tìm đường đi...",
            "PAUSED (SPACE để tiếp tục)" if paused else "",
            "Ẩn cây (T để bật/tắt)" if not show_tree else "",
            "Ẩn animation (A để bật/tắt)" if not show_animation else ""
        ]
        
        for i, text in enumerate(info_texts):
            if text:  # Chỉ hiển thị nếu text không rỗng
                text_surface = font.render(text, True, BLACK) if i < 4 else small_font.render(text, True, BLACK)
                screen.blit(text_surface, (10, 10 + i * 25))
        
        pygame.display.flip()
        clock.tick(60)
    
    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()