import numpy as np
import matplotlib.pyplot as plt

class Boid:
    def __init__(self, x, y, vx, vy, max_speed, max_force):
        self.position = np.array([x, y])
        self.velocity = np.array([vx, vy])
        self.max_speed = max_speed
        self.max_force = max_force

    def update(self, boids):
        alignment_radius = 50
        separation_radius = 25
        cohesion_radius = 50
        max_speed = 2
        max_force = 0.1

        alignment = np.zeros(2)
        separation = np.zeros(2)
        cohesion = np.zeros(2)
        count_alignment = 0
        count_cohesion = 0

        for other in boids:
            distance = np.linalg.norm(self.position - other.position)

            if 0 < distance < alignment_radius:
                alignment += other.velocity
                count_alignment += 1

            if 0 < distance < separation_radius:
                diff = self.position - other.position
                diff /= distance
                separation += diff

            if 0 < distance < cohesion_radius:
                cohesion += other.position
                count_cohesion += 1

        if count_alignment > 0:
            alignment /= count_alignment
            alignment /= np.linalg.norm(alignment)
            alignment *= max_speed

        if count_cohesion > 0:
            cohesion /= count_cohesion
            cohesion = self.steer(cohesion)

        alignment *= 1.0
        separation *= 1.5
        cohesion *= 1.0

        acceleration = alignment + separation + cohesion
        self.velocity += acceleration
        self.velocity = self.limit(self.velocity, max_speed)
        self.position += self.velocity

    def steer(self, target):
        desired = target - self.position
        desired /= np.linalg.norm(desired)
        desired *= self.max_speed
        steer = desired - self.velocity
        steer = self.limit(steer, self.max_force)  # Sử dụng self.max_force
        return steer

    def limit(self, vector, max_value):
        magnitude = np.linalg.norm(vector)
        if magnitude > max_value:
            vector /= magnitude
            vector *= max_value
        return vector

    def draw(self):
        plt.scatter(self.position[0], self.position[1], color='black')

# Khởi tạo đám đông ban đầu
num_boids = 100
boids = []
max_speed = 2
max_force = 0.1
for _ in range(num_boids):
    x = np.random.uniform(0, 800)
    y = np.random.uniform(0, 600)
    vx = np.random.uniform(-1, 1)
    vy = np.random.uniform(-1, 1)
    boid = Boid(x, y, vx, vy, max_speed, max_force)
    boids.append(boid)

# Mô phỏng hành vi tụ tập của đám đông
num_steps = 500
for step in range(num_steps):
    plt.clf()
    for boid in boids:
        boid.update(boids)
        boid.draw()
    plt.xlim(0, 800)
    plt.ylim(0, 600)
    plt.pause(0.01)

plt.show()
