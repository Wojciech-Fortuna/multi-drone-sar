import random
import numpy as np

from controllers.metrics import trajectory_distance


class GeneticAlgorithmPlanner:
    def __init__(
        self,
        terrain,
        probability_map,
        candidate_points,
        visibility_sets,
        start_pos,
        time_budget=120.0,
        v_max=5.0,
        population_size=40,
        chromosome_length=8,
        mutation_rate=0.2,
        crossover_rate=0.8,
        elite_size=2,
        distance_penalty=0.001
    ):
        self.terrain = terrain
        self.probability_map = probability_map
        self.candidate_points = candidate_points
        self.visibility_sets = visibility_sets
        self.start_pos = np.asarray(start_pos, dtype=float)

        self.time_budget = time_budget
        self.v_max = v_max

        self.population_size = population_size
        self.chromosome_length = chromosome_length
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.elite_size = elite_size
        self.distance_penalty = distance_penalty

    def create_random_individual(self):
        return [random.randrange(len(self.candidate_points)) for _ in range(self.chromosome_length)]

    def initialize_population(self):
        return [self.create_random_individual() for _ in range(self.population_size)]

    def decode_trajectory(self, individual):
        trajectory = [self.start_pos]

        for candidate_idx in individual:
            trajectory.append(self.candidate_points[candidate_idx])

        return trajectory

    def is_time_feasible(self, trajectory):
        total_time = trajectory_distance(trajectory) / self.v_max
        return total_time <= self.time_budget

    def repair_individual(self, individual):
        repaired = list(individual)

        while len(repaired) > 0:
            trajectory = self.decode_trajectory(repaired)

            if self.is_time_feasible(trajectory):
                break

            repaired.pop()

        return repaired

    def fitness(self, individual):
        individual = self.repair_individual(individual)

        observed = set()

        for candidate_idx in individual:
            visible_cells = self.visibility_sets[candidate_idx]

            for cell_idx in visible_cells:
                observed.add(cell_idx)

        covered_probability = sum(
            self.probability_map.probabilities[cell_idx]
            for cell_idx in observed
        )

        trajectory = self.decode_trajectory(individual)
        distance = trajectory_distance(trajectory)

        return covered_probability - self.distance_penalty * distance

    def tournament_selection(self, population, fitness_values, tournament_size=3):
        selected_indices = random.sample(range(len(population)), tournament_size)

        best_idx = max(selected_indices, key=lambda idx: fitness_values[idx])

        return list(population[best_idx])

    def crossover(self, parent1, parent2):
        if random.random() > self.crossover_rate:
            return list(parent1), list(parent2)

        if len(parent1) < 2 or len(parent2) < 2:
            return list(parent1), list(parent2)

        cut = random.randint(1, min(len(parent1), len(parent2)) - 1)

        child1 = parent1[:cut] + parent2[cut:]
        child2 = parent2[:cut] + parent1[cut:]

        return child1, child2

    def mutate(self, individual):
        mutated = list(individual)

        for i in range(len(mutated)):
            if random.random() < self.mutation_rate:
                mutated[i] = random.randrange(len(self.candidate_points))

        return mutated

    def evolve(self, generations=50):
        population = self.initialize_population()

        best_individual = None
        best_fitness = -float("inf")
        history = []

        for generation in range(generations):
            fitness_values = [self.fitness(individual) for individual in population]

            generation_best_idx = int(np.argmax(fitness_values))
            generation_best_fitness = fitness_values[generation_best_idx]

            if generation_best_fitness > best_fitness:
                best_fitness = generation_best_fitness
                best_individual = list(population[generation_best_idx])

            history.append(best_fitness)

            print(f"Generation {generation + 1}/{generations}, "f"best fitness: {best_fitness:.4f}")

            sorted_indices = np.argsort(fitness_values)[::-1]

            new_population = [list(population[idx]) for idx in sorted_indices[:self.elite_size]]

            while len(new_population) < self.population_size:
                parent1 = self.tournament_selection(population, fitness_values)

                parent2 = self.tournament_selection(population, fitness_values)

                child1, child2 = self.crossover(parent1, parent2)

                child1 = self.mutate(child1)
                child2 = self.mutate(child2)

                child1 = self.repair_individual(child1)
                child2 = self.repair_individual(child2)

                new_population.append(child1)

                if len(new_population) < self.population_size:
                    new_population.append(child2)

            population = new_population

        best_individual = self.repair_individual(best_individual)
        best_trajectory = self.decode_trajectory(best_individual)

        return {
            "best_individual": best_individual,
            "best_trajectory": best_trajectory,
            "best_fitness": best_fitness,
            "history": history
        }

class MultiDroneGeneticAlgorithmPlanner:
    def __init__(
        self,
        terrain,
        probability_map,
        candidate_points,
        visibility_sets,
        start_positions,
        time_budget=120.0,
        v_max=5.0,
        population_size=40,
        chromosome_length=12,
        mutation_rate=0.2,
        crossover_rate=0.8,
        elite_size=2,
        distance_penalty=0.001,
        time_penalty=0.01
    ):
        self.terrain = terrain
        self.probability_map = probability_map
        self.candidate_points = candidate_points
        self.visibility_sets = visibility_sets

        self.start_positions = [np.asarray(pos, dtype=float) for pos in start_positions]

        self.num_drones = len(start_positions)
        self.time_budget = time_budget
        self.v_max = v_max

        self.population_size = population_size
        self.chromosome_length = chromosome_length
        self.mutation_rate = mutation_rate
        self.crossover_rate = crossover_rate
        self.elite_size = elite_size
        self.distance_penalty = distance_penalty
        self.time_penalty = time_penalty

    def create_random_gene(self):
        drone_idx = random.randrange(self.num_drones)
        candidate_idx = random.randrange(len(self.candidate_points))

        return (drone_idx, candidate_idx)

    def create_random_individual(self):
        return [self.create_random_gene() for _ in range(self.chromosome_length)]

    def initialize_population(self):
        return [self.create_random_individual() for _ in range(self.population_size)]

    def decode_trajectories(self, individual):
        trajectories = [[start_pos] for start_pos in self.start_positions]

        for drone_idx, candidate_idx in individual:
            waypoint = self.candidate_points[candidate_idx]
            trajectories[drone_idx].append(waypoint)

        return trajectories

    def trajectory_distance(self, trajectory):
        trajectory = np.asarray(trajectory, dtype=float)

        if len(trajectory) < 2:
            return 0.0

        distance = 0.0

        for i in range(1, len(trajectory)):
            distance += np.linalg.norm(trajectory[i] - trajectory[i - 1])

        return distance

    def compute_drone_times(self, trajectories):
        return [
            self.trajectory_distance(trajectory) / self.v_max
            for trajectory in trajectories
        ]

    def total_distance(self, trajectories):
        return sum(
            self.trajectory_distance(trajectory)
            for trajectory in trajectories
        )

    def repair_individual(self, individual):
        repaired = list(individual)

        while len(repaired) > 0:
            trajectories = self.decode_trajectories(repaired)
            drone_times = self.compute_drone_times(trajectories)

            if all(time <= self.time_budget for time in drone_times):
                break

            repaired.pop()

        return repaired

    def evaluate_solution(self, individual):
        individual = self.repair_individual(individual)

        observed = set()

        for _, candidate_idx in individual:
            visible_cells = self.visibility_sets[candidate_idx]

            for cell_idx in visible_cells:
                observed.add(cell_idx)

        covered_probability = sum(
            self.probability_map.probabilities[cell_idx]
            for cell_idx in observed
        )

        trajectories = self.decode_trajectories(individual)
        drone_times = self.compute_drone_times(trajectories)
        distance = self.total_distance(trajectories)

        max_time_excess = max(
            max(0.0, time - self.time_budget)
            for time in drone_times
        )

        fitness = (covered_probability
            - self.distance_penalty * distance
            - self.time_penalty * max_time_excess
        )

        return {
            "fitness": fitness,
            "covered_probability": covered_probability,
            "distance": distance,
            "drone_times": drone_times,
            "trajectories": trajectories,
            "num_waypoints": sum(len(t) for t in trajectories)
        }

    def fitness(self, individual):
        return self.evaluate_solution(individual)["fitness"]

    def tournament_selection(
        self,
        population,
        fitness_values,
        tournament_size=3
    ):
        selected_indices = random.sample(range(len(population)), tournament_size)

        best_idx = max(selected_indices, key=lambda idx: fitness_values[idx])

        return list(population[best_idx])

    def crossover(self, parent1, parent2):
        if random.random() > self.crossover_rate:
            return list(parent1), list(parent2)

        if len(parent1) < 2 or len(parent2) < 2:
            return list(parent1), list(parent2)

        cut = random.randint(1, min(len(parent1), len(parent2)) - 1)

        child1 = parent1[:cut] + parent2[cut:]
        child2 = parent2[:cut] + parent1[cut:]

        return child1, child2

    def mutate(self, individual):
        mutated = list(individual)

        for i in range(len(mutated)):
            if random.random() < self.mutation_rate:
                mutation_type = random.choice(["change_drone", "change_candidate", "change_both"])

                drone_idx, candidate_idx = mutated[i]

                if mutation_type == "change_drone":
                    drone_idx = random.randrange(self.num_drones)

                elif mutation_type == "change_candidate":
                    candidate_idx = random.randrange(len(self.candidate_points))

                else:
                    drone_idx = random.randrange(self.num_drones)
                    candidate_idx = random.randrange(len(self.candidate_points))

                mutated[i] = (drone_idx, candidate_idx)

        return mutated

    def evolve(self, generations=50):
        population = self.initialize_population()

        best_individual = None
        best_fitness = -float("inf")
        history = []

        for generation in range(generations):
            fitness_values = [self.fitness(individual) for individual in population]

            generation_best_idx = int(np.argmax(fitness_values))
            generation_best_fitness = fitness_values[generation_best_idx]

            if generation_best_fitness > best_fitness:
                best_fitness = generation_best_fitness
                best_individual = list(population[generation_best_idx])

            history.append(best_fitness)

            print(f"Generation {generation + 1}/{generations}, "f"best fitness: {best_fitness:.4f}")

            sorted_indices = np.argsort(fitness_values)[::-1]

            new_population = [list(population[idx]) for idx in sorted_indices[:self.elite_size]]

            while len(new_population) < self.population_size:
                parent1 = self.tournament_selection(population, fitness_values)
                parent2 = self.tournament_selection(population, fitness_values)

                child1, child2 = self.crossover(parent1, parent2)

                child1 = self.mutate(child1)
                child2 = self.mutate(child2)

                child1 = self.repair_individual(child1)
                child2 = self.repair_individual(child2)

                new_population.append(child1)

                if len(new_population) < self.population_size:
                    new_population.append(child2)

            population = new_population

        best_individual = self.repair_individual(best_individual)
        final_metrics = self.evaluate_solution(best_individual)

        return {
            "best_individual": best_individual,
            "best_fitness": best_fitness,
            "history": history,
            "trajectories": final_metrics["trajectories"],
            "covered_probability": final_metrics["covered_probability"],
            "distance": final_metrics["distance"],
            "drone_times": final_metrics["drone_times"],
            "num_waypoints": final_metrics["num_waypoints"]
        }
