#This code shows an algorithm that uses physics to determine the most optimum path for vehicles within a collision zone.

import numpy as np
import csv
import heapq
import collections

# ==========================================
# 1. Core Data Structures
# ==========================================
class Vehicle:
    def __init__(self, v_id, start, end, speed, mass_priority, margin=3.0, start_time=0.0):
        self.v_id = str(v_id)
        self.start = np.array(start, dtype=float)
        self.end = np.array(end, dtype=float)
        self.speed = float(speed)
        self.priority = float(mass_priority)
        self.margin = float(margin)
        self.start_time = float(start_time)

        self.momentum = self.priority * self.speed

        self.is_active = False
        self.reached_goal = False
        self.state = 'NORMAL'
        self.needs_replan = True

        self.current_pos = np.copy(self.start)
        self.prop_pos = np.copy(self.start)
        self.last_pos = np.copy(self.start)

        dist = np.linalg.norm(self.end - self.start)
        self.last_dir = (self.end - self.start) / dist if dist > 1e-5 else np.array([1.0, 0.0, 0.0])

        self.waypoints = collections.deque()
        self.deflection_target = None

        self.original_path = []
        self.resolved_path = [{'t': 0.0, 'pos': np.copy(self.start)}]
        self.state_history = ['NORMAL']

        self.orig_dist = np.linalg.norm(self.end - self.start)
        self.orig_time = self.orig_dist / self.speed
        self.new_time = 0.0

class CylinderObstacle:
    def __init__(self, x, y, radius, z_min=0, z_max=100):
        self.x = x
        self.y = y
        self.radius = radius
        self.z_min = z_min
        self.z_max = z_max

# ==========================================
# 2. Local Conditional 2D/3D A* Replanning Engine
# ==========================================
class FramedAStarEngine:
    def __init__(self, grid_res=3.0, allow_3d=False):
        self.grid_res = grid_res
        self.allow_3d = allow_3d

    def compute_path(self, v, start_pos, goal_pos, static_obstacles, dynamic_walls):
        def to_grid(p): return (int(round(p[0]/self.grid_res)), int(round(p[1]/self.grid_res)), int(round(p[2]/self.grid_res)))
        def to_world(g): return np.array([g[0]*self.grid_res, g[1]*self.grid_res, g[2]*self.grid_res], dtype=float)

        effective_goal = np.copy(goal_pos)
        if not self.allow_3d:
            effective_goal[2] = start_pos[2]

        start_g = to_grid(start_pos)
        goal_g = to_grid(effective_goal)

        if start_g == goal_g: return [np.copy(effective_goal)]

        open_set = []
        heapq.heappush(open_set, (0, start_g))
        came_from = {}
        g_score = {start_g: 0}

        closest_node = start_g
        min_dist_to_goal = np.linalg.norm(to_world(start_g) - effective_goal)

        def is_safe(grid_node):
            w = to_world(grid_node)
            if w[2] < 0: return False

            for obs in static_obstacles:
                if obs.z_min <= w[2] <= obs.z_max:
                    dist = np.linalg.norm(w[:2] - np.array([obs.x, obs.y]))
                    if dist < (obs.radius + v.margin + 0.5): return False

            for wall_pos, wall_margin in dynamic_walls:
                if np.linalg.norm(w - wall_pos) < (v.margin + wall_margin - 0.5):
                    return False
            return True

        iterations = 0

        dz_options = [-1, 0, 1] if self.allow_3d else [0]

        while open_set and iterations < 6000:
            iterations += 1
            _, current = heapq.heappop(open_set)

            curr_world = to_world(current)
            dist_to_goal = np.linalg.norm(curr_world - effective_goal)

            if dist_to_goal < min_dist_to_goal:
                min_dist_to_goal = dist_to_goal
                closest_node = current

            if current == goal_g or dist_to_goal < self.grid_res * 1.5:
                path = [np.copy(effective_goal)]
                while current in came_from:
                    current = came_from[current]
                    if current != start_g:
                        path.append(to_world(current))
                path.reverse()
                return path

            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    for dz in dz_options:
                        if dx==0 and dy==0 and dz==0: continue
                        nxt = (current[0]+dx, current[1]+dy, current[2]+dz)

                        if not is_safe(nxt): continue

                        cost = np.linalg.norm([dx, dy, dz]) * self.grid_res
                        tent_g = g_score[current] + cost
                        if nxt not in g_score or tent_g < g_score[nxt]:
                            g_score[nxt] = tent_g
                            came_from[nxt] = current

                            h = np.linalg.norm(to_world(nxt) - effective_goal)
                            f = tent_g + (1.5 * h)
                            heapq.heappush(open_set, (f, nxt))

        path = []
        curr = closest_node
        while curr in came_from:
            curr = came_from[curr]
            if curr != start_g:
                path.append(to_world(curr))
        path.reverse()
        if not path:
            return [np.copy(effective_goal)]
        return path

# ==========================================
# 3. Environment-Aware 90-Degree Deflection
# ==========================================
def calculate_90_deg_deflection(v_target, v_opposite, obstacles):
    dir_i = v_target.last_dir
    left_90 = np.array([-dir_i[1], dir_i[0], 0.0])
    right_90 = np.array([dir_i[1], -dir_i[0], 0.0])

    evasion_dir = v_target.current_pos - v_opposite.current_pos
    evasion_dir[2] = 0

    for obs in obstacles:
        if obs.z_min <= v_target.current_pos[2] <= obs.z_max:
            vec2d = v_target.current_pos[:2] - np.array([obs.x, obs.y])
            dist2d = np.linalg.norm(vec2d)
            if dist2d < (obs.radius + v_target.margin + 5.0):
                repel = vec2d / dist2d if dist2d > 1e-3 else np.array([1.0, 0.0])
                weight = (obs.radius + v_target.margin + 5.0) - dist2d
                evasion_dir[:2] += repel * weight * 2.0

    norm_ev = np.linalg.norm(evasion_dir)
    evasion_dir = evasion_dir / norm_ev if norm_ev > 1e-3 else np.array([1.0, 0.0, 0.0])

    goal_dir = v_target.end - v_target.current_pos
    goal_dir[2] = 0
    norm_goal = np.linalg.norm(goal_dir)
    goal_dir = goal_dir / norm_goal if norm_goal > 1e-3 else dir_i

    score_left = (3.0 * np.dot(left_90, evasion_dir)) + (1.0 * np.dot(left_90, goal_dir))
    score_right = (3.0 * np.dot(right_90, evasion_dir)) + (1.0 * np.dot(right_90, goal_dir))

    return left_90 if score_left > score_right else right_90

# ==========================================
# 4. Hybrid Simulation Engine
# ==========================================
class Simulation:
    def __init__(self, astar_frame_size=25.0, allow_3d=False, localized_pbd=True):
        self.vehicles = []
        self.obstacles = []
        self.dt = 0.2
        self.astar_frame_size = astar_frame_size
        self.allow_3d = allow_3d
        self.localized_pbd = localized_pbd

        self.astar = FramedAStarEngine(grid_res=3.0, allow_3d=self.allow_3d)
        self.deflection_events = []

    def add_vehicle(self, v: Vehicle):
        self.vehicles.append(v)

    def add_obstacle(self, obs: CylinderObstacle):
        self.obstacles.append(obs)

    def run(self):
        t = 0.0
        max_t = 300.0
        safety_buffer = 0.2

        for v in self.vehicles:
            steps = int(np.ceil(v.orig_time / self.dt))
            for i in range(steps + 1):
                fraction = min((i * self.dt) / v.orig_time, 1.0)
                v.original_path.append({'t': v.start_time + (i * self.dt), 'pos': v.start + fraction * (v.end - v.start)})

        while t < max_t:
            t += self.dt

            # ---------------------------------------------------------
            # PHASE 1: State Management & Intent
            # ---------------------------------------------------------
            for v in self.vehicles:
                if not v.is_active:
                    if t >= v.start_time: v.is_active = True
                    else:
                        v.prop_pos = np.copy(v.current_pos)
                        continue

                dist_to_goal = np.linalg.norm(v.end - v.current_pos)
                if dist_to_goal < 0.5:
                    v.reached_goal = True
                    v.state = 'NORMAL'
                    v.prop_pos = np.copy(v.current_pos)
                    continue
                else:
                    v.reached_goal = False

                if v.state == 'DEFLECTING':
                    actual_movement = np.linalg.norm(v.current_pos - v.last_pos)
                    dist_to_target = np.linalg.norm(v.deflection_target - v.current_pos)

                    if dist_to_target < 0.5 or actual_movement < 0.05:
                        v.state = 'NORMAL'
                        v.needs_replan = True
                    else:
                        direction = (v.deflection_target - v.current_pos) / dist_to_target
                        v.last_dir = direction
                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, dist_to_target)

                if v.state == 'NORMAL':
                    if v.needs_replan:
                        dynamic_walls = []
                        for other in self.vehicles:
                            if other.v_id != v.v_id and not other.reached_goal and other.is_active:
                                if np.linalg.norm(other.current_pos - v.current_pos) <= self.astar_frame_size:
                                    dynamic_walls.append((np.copy(other.current_pos), other.margin))

                        path = self.astar.compute_path(v, v.current_pos, v.end, self.obstacles, dynamic_walls)
                        v.waypoints = collections.deque(path)
                        v.needs_replan = False

                    if len(v.waypoints) > 0:
                        target = v.waypoints[0]
                        vec = target - v.current_pos
                        dist = np.linalg.norm(vec)
                        if dist < 0.5:
                            v.waypoints.popleft()
                            if len(v.waypoints) > 0:
                                target = v.waypoints[0]
                                vec = target - v.current_pos
                                dist = np.linalg.norm(vec)

                        if dist > 1e-5:
                            direction = vec / dist
                            v.last_dir = direction
                        else:
                            direction = np.zeros(3)

                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, dist)
                    else:
                        if dist_to_goal > 1e-5:
                            direction = (v.end - v.current_pos) / dist_to_goal
                            v.last_dir = direction
                        else:
                            direction = np.zeros(3)
                        v.prop_pos = v.current_pos + direction * min(v.speed * self.dt, dist_to_goal)

            active_veh = [v for v in self.vehicles if v.is_active and not v.reached_goal]
            n_veh = len(active_veh)

            # ---------------------------------------------------------
            # PHASE 2: Collision Trigger -> Evasion-Biased Deflection
            # ---------------------------------------------------------
            for i in range(n_veh):
                vi = active_veh[i]
                for j in range(i+1, n_veh):
                    vj = active_veh[j]

                    req_dist = vi.margin + vj.margin + safety_buffer
                    dist = np.linalg.norm(vi.prop_pos - vj.prop_pos)

                    if dist < req_dist and (vi.state == 'NORMAL' or vj.state == 'NORMAL'):
                        sum_mom = vi.momentum + vj.momentum

                        if vi.state == 'NORMAL':
                            dir_i = calculate_90_deg_deflection(vi, vj, self.obstacles)
                            dist_i = max((vj.momentum / sum_mom) * (vi.speed * 4.0), vi.margin * 1.5)
                            vi.deflection_target = vi.current_pos + dir_i * dist_i
                            vi.state = 'DEFLECTING'
                            vi.waypoints.clear()

                            vi.prop_pos = vi.current_pos + dir_i * min(vi.speed * self.dt, dist_i)
                            self.deflection_events.append({'v_id': vi.v_id, 'pos': np.copy(vi.current_pos)})

                        if vj.state == 'NORMAL':
                            dir_j = calculate_90_deg_deflection(vj, vi, self.obstacles)
                            dist_j = max((vi.momentum / sum_mom) * (vj.speed * 4.0), vj.margin * 1.5)
                            vj.deflection_target = vj.current_pos + dir_j * dist_j
                            vj.state = 'DEFLECTING'
                            vj.waypoints.clear()

                            vj.prop_pos = vj.current_pos + dir_j * min(vj.speed * self.dt, dist_j)
                            self.deflection_events.append({'v_id': vj.v_id, 'pos': np.copy(vj.current_pos)})

            # ---------------------------------------------------------
            # PHASE 3: TOGGLEABLE MICRO-PBD (Broad-Phase vs Global)
            # ---------------------------------------------------------
            if self.localized_pbd:
                danger_pairs = []
                for i in range(n_veh):
                    vi = active_veh[i]
                    for j in range(i+1, n_veh):
                        vj = active_veh[j]
                        dist = np.linalg.norm(vi.prop_pos - vj.prop_pos)
                        if dist <= self.astar_frame_size:
                            danger_pairs.append((vi, vj))

                for _ in range(30):
                    for vi, vj in danger_pairs:
                        req_dist = vi.margin + vj.margin + safety_buffer
                        dist = np.linalg.norm(vi.prop_pos - vj.prop_pos)
                        if dist < req_dist:
                            overlap = req_dist - dist
                            dir = (vi.prop_pos - vj.prop_pos) / dist if dist > 1e-5 else np.array([1.0, 0.0, 0.0])

                            sum_mom = vi.momentum + vj.momentum
                            vi.prop_pos += dir * (overlap * (vj.momentum / sum_mom) * 0.5)
                            vj.prop_pos -= dir * (overlap * (vi.momentum / sum_mom) * 0.5)

                    for v in active_veh:
                        for obs in self.obstacles:
                            if obs.z_min <= v.prop_pos[2] <= obs.z_max:
                                vec2d = v.prop_pos[:2] - np.array([obs.x, obs.y])
                                dist2d = np.linalg.norm(vec2d)
                                req_dist = obs.radius + v.margin + safety_buffer
                                if dist2d < req_dist:
                                    overlap = req_dist - dist2d
                                    dir2d = vec2d / dist2d if dist2d > 1e-5 else np.array([1.0, 0.0])
                                    v.prop_pos[:2] += dir2d * overlap
            else:
                for _ in range(30):
                    for i in range(n_veh):
                        vi = active_veh[i]
                        for j in range(i+1, n_veh):
                            vj = active_veh[j]
                            req_dist = vi.margin + vj.margin + safety_buffer
                            dist = np.linalg.norm(vi.prop_pos - vj.prop_pos)
                            if dist < req_dist:
                                overlap = req_dist - dist
                                dir = (vi.prop_pos - vj.prop_pos) / dist if dist > 1e-5 else np.array([1.0, 0.0, 0.0])

                                sum_mom = vi.momentum + vj.momentum
                                vi.prop_pos += dir * (overlap * (vj.momentum / sum_mom) * 0.5)
                                vj.prop_pos -= dir * (overlap * (vi.momentum / sum_mom) * 0.5)

                    for v in active_veh:
                        for obs in self.obstacles:
                            if obs.z_min <= v.prop_pos[2] <= obs.z_max:
                                vec2d = v.prop_pos[:2] - np.array([obs.x, obs.y])
                                dist2d = np.linalg.norm(vec2d)
                                req_dist = obs.radius + v.margin + safety_buffer
                                if dist2d < req_dist:
                                    overlap = req_dist - dist2d
                                    dir2d = vec2d / dist2d if dist2d > 1e-5 else np.array([1.0, 0.0])
                                    v.prop_pos[:2] += dir2d * overlap

            # ---------------------------------------------------------
            # PHASE 4: Commit State
            # ---------------------------------------------------------
            for v in self.vehicles:
                if not v.is_active: continue
                v.last_pos = np.copy(v.current_pos)
                v.current_pos = np.copy(v.prop_pos)
                v.resolved_path.append({'t': t, 'pos': np.copy(v.current_pos)})
                v.state_history.append(v.state)

                if v.reached_goal and v.new_time == 0.0:
                    v.new_time = t

        self.verify_manifest()
        self.export_csv()

# ==========================================
# 5. Manifest Evaluation Checker
# ==========================================
    def verify_manifest(self):
        print("\n" + "="*80)
        print(f"SIMULATION MANIFEST VERIFICATION (TIME/DELAY ANALYSIS)")
        print(f"PBD Mode: {'LOCALIZED (A* Frame)' if self.localized_pbd else 'GLOBAL (Entire Map)'}")
        print("="*80)

        all_reached = True
        for v in self.vehicles:
            final_pos = v.resolved_path[-1]['pos']
            dist_to_goal = np.linalg.norm(final_pos - v.end)
            if dist_to_goal > 0.5:
                print(f"❌ Veh {v.v_id:<10} FAILED to reach destination. Final Dist: {dist_to_goal:.2f}m")
                all_reached = False
            else:
                actual_duration = v.new_time - v.start_time
                delay = actual_duration - v.orig_time
                # Handle edge cases where due to arrival thresholds, delay is technically negative
                if delay < 0: delay = 0.0
                print(f"✅ Veh {v.v_id:<10} reached. [Ideal: {v.orig_time:>5.2f}s | Actual: {actual_duration:>5.2f}s | Delay: +{delay:>5.2f}s]")

        if all_reached:
            print("\n🟢 ALL drones successfully reached their destinations.")

        print("-" * 80)
        print("COLLISION MARGIN CHECK (Center-to-Center)")

        v_paths = {v.v_id: {round(wp['t'], 2): wp['pos'] for wp in v.resolved_path} for v in self.vehicles}
        timestamps = sorted(list(set(round(wp['t'], 2) for v in self.vehicles for wp in v.resolved_path)))

        violations = 0
        for t in timestamps:
            active_at_t = []
            for v in self.vehicles:
                if t > v.start_time and (v.new_time == 0.0 or t <= v.new_time):
                    if np.linalg.norm(v_paths[v.v_id][t] - v.end) > 0.5:
                        active_at_t.append((v, v_paths[v.v_id][t]))

            for i in range(len(active_at_t)):
                for j in range(i+1, len(active_at_t)):
                    v1, pos1 = active_at_t[i]
                    v2, pos2 = active_at_t[j]

                    min_allowed = v1.margin + v2.margin - 0.05
                    dist = np.linalg.norm(pos1 - pos2)

                    if dist < min_allowed:
                        violations += 1
                        print(f"⚠️ VIOLATION at t={t:.1f}s | Veh {v1.v_id} & Veh {v2.v_id} | Dist: {dist:.2f}m (Allowed: {v1.margin+v2.margin:.2f}m)")

        if violations == 0:
            print("🟢 ZERO margin violations detected. Absolute safety maintained.")
        else:
            print(f"🔴 WARNING: {violations} mid-air margin violations detected.")
        print("="*80 + "\n")

    def export_csv(self):
        filename = "resolved_paths.csv"
        with open(filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Vehicle_ID', 'Margin', 'Time', 'X', 'Y', 'Z'])
            for v in self.vehicles:
                for wp in v.resolved_path:
                    pos = wp['pos']
                    writer.writerow([v.v_id, v.margin, round(wp['t'],2), round(pos[0],2), round(pos[1],2), round(pos[2],2)])

# ==========================================
# 6. Isolate-Group Plotly Renderer
# ==========================================
    def plot(self):
        try:
            import plotly.graph_objects as go
        except ImportError:
            print("\n❌ Error: Plotly is not installed.\n")
            return

        fig = go.Figure()

        for obs in self.obstacles:
            z = np.linspace(obs.z_min, obs.z_max, 10)
            theta = np.linspace(0, 2*np.pi, 20)
            theta_grid, z_grid = np.meshgrid(theta, z)
            x_grid = obs.radius * np.cos(theta_grid) + obs.x
            y_grid = obs.radius * np.sin(theta_grid) + obs.y
            fig.add_trace(go.Surface(x=x_grid, y=y_grid, z=z_grid, colorscale='Greys', opacity=0.8, showscale=False, hoverinfo='skip'))

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b', '#e377c2', '#17becf', '#bcbd22']
        dynamic_trace_indices = []

        for idx, v in enumerate(self.vehicles):
            color = colors[idx % len(colors)]
            grp = f"Veh {v.v_id}"

            orig_pts = np.array([wp['pos'] for wp in v.original_path])
            fig.add_trace(go.Scatter3d(x=orig_pts[:,0], y=orig_pts[:,1], z=orig_pts[:,2], mode='lines', line=dict(color=color, dash='dash', width=2), opacity=0.3, legendgroup=grp, showlegend=False, hoverinfo='skip'))

            res_pts = np.array([wp['pos'] for wp in v.resolved_path])
            fig.add_trace(go.Scatter3d(x=res_pts[:,0], y=res_pts[:,1], z=res_pts[:,2], mode='lines', line=dict(color=color, width=3), name=grp, legendgroup=grp, showlegend=True))

            fig.add_trace(go.Scatter3d(x=[v.start[0]], y=[v.start[1]], z=[v.start[2]], mode='markers+text', marker=dict(size=5, color='lightgreen', line=dict(width=1, color='green')), text=[f"Start {v.v_id}"], textposition="top center", legendgroup=grp, showlegend=False))
            fig.add_trace(go.Scatter3d(x=[v.end[0]], y=[v.end[1]], z=[v.end[2]], mode='markers+text', marker=dict(size=5, color='salmon', symbol='x', line=dict(width=1, color='red')), text=[f"End {v.v_id}"], textposition="bottom center", legendgroup=grp, showlegend=False))

            v_deflections = [e['pos'] for e in self.deflection_events if e['v_id'] == v.v_id]
            if v_deflections:
                dx = [p[0] for p in v_deflections]; dy = [p[1] for p in v_deflections]; dz = [p[2] for p in v_deflections]
                fig.add_trace(go.Scatter3d(x=dx, y=dy, z=dz, mode='markers', marker=dict(size=5, color='yellow', symbol='diamond', line=dict(color='orange', width=2)), legendgroup=grp, showlegend=False, hoverinfo='skip'))

        for idx, v in enumerate(self.vehicles):
            color = colors[idx % len(colors)]
            grp = f"Veh {v.v_id}"

            marker_idx = len(fig.data)
            fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode='markers', marker=dict(size=7, color=color, line=dict(width=1, color='black')), legendgroup=grp, showlegend=False))
            dynamic_trace_indices.append(marker_idx)

            bubble_idx = len(fig.data)
            fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode='lines', line=dict(color='red', width=3), opacity=0.5, legendgroup=grp, showlegend=False, hoverinfo='skip'))
            dynamic_trace_indices.append(bubble_idx)

            grid_idx = len(fig.data)
            fig.add_trace(go.Scatter3d(x=[], y=[], z=[], mode='lines', line=dict(color=color, width=1, dash='dot'), opacity=0.4, legendgroup=grp, showlegend=False, hoverinfo='skip'))
            dynamic_trace_indices.append(grid_idx)

        frames = []
        theta_bubble = np.linspace(0, 2 * np.pi, 25)
        frames_total = max([len(v.resolved_path) for v in self.vehicles])

        for i in range(frames_total):
            frame_data = []
            for j, v in enumerate(self.vehicles):
                current_time = i * self.dt
                if current_time < v.start_time:
                    frame_data.append(go.Scatter3d(x=[None], y=[None], z=[None]))
                    frame_data.append(go.Scatter3d(x=[None], y=[None], z=[None]))
                    frame_data.append(go.Scatter3d(x=[None], y=[None], z=[None]))
                    continue

                idx = min(i, len(v.resolved_path) - 1)
                pos = v.resolved_path[idx]['pos']

                if np.linalg.norm(pos - v.end) < 0.5:
                    m_color = '#cccccc'
                else:
                    m_color = colors[j % len(colors)]

                frame_data.append(go.Scatter3d(x=[pos[0]], y=[pos[1]], z=[pos[2]], marker=dict(color=m_color)))

                mx = (pos[0] + v.margin * np.cos(theta_bubble)).tolist() + [None]
                my = (pos[1] + v.margin * np.sin(theta_bubble)).tolist() + [None]
                mz = (np.full_like(theta_bubble, pos[2])).tolist() + [None]
                frame_data.append(go.Scatter3d(x=mx, y=my, z=mz))

                gx = (pos[0] + self.astar_frame_size * np.cos(theta_bubble)).tolist() + [None]
                gy = (pos[1] + self.astar_frame_size * np.sin(theta_bubble)).tolist() + [None]
                gz = (np.full_like(theta_bubble, pos[2])).tolist() + [None]
                frame_data.append(go.Scatter3d(x=gx, y=gy, z=gz))

            frames.append(go.Frame(data=frame_data, name=f"{i*self.dt:.1f}", traces=dynamic_trace_indices))

        fig.frames = frames

        sliders = [{'pad': {'b': 10, 't': 50}, 'len': 0.9, 'x': 0.1, 'y': 0, 'currentvalue': {'prefix': 'Time: ', 'suffix': 's'},
                    'steps': [{'args': [[f.name], {'frame': {'duration': 0, 'redraw': True}, 'mode': 'immediate'}], 'label': f.name, 'method': 'animate'} for f in fig.frames[::2]]}]

        pbd_title_text = "LOCALIZED (A* Grid)" if self.localized_pbd else "GLOBAL (Entire Map)"

        fig.update_layout(
            title=f"A* Routing & PBD [{pbd_title_text}] - Double-Click Legend to Isolate",
            plot_bgcolor='white', paper_bgcolor='white',
            scene=dict(xaxis=dict(showbackground=False, showgrid=True), yaxis=dict(showbackground=False, showgrid=True), zaxis=dict(showbackground=False, showgrid=True), aspectmode='data'),
            updatemenus=[{'type': 'buttons', 'showactive': False, 'y': 0, 'x': 0.05, 'xanchor': 'right', 'yanchor': 'top', 'pad': {'t': 50, 'r': 10},
                'buttons': [{'label': '▶ Play', 'method': 'animate', 'args': [None, {'frame': {'duration': 40, 'redraw': True}, 'fromcurrent': True, 'transition': {'duration': 0}}]},
                            {'label': '⏸ Pause', 'method': 'animate', 'args': [[None], {'frame': {'duration': 0, 'redraw': False}, 'mode': 'immediate', 'transition': {'duration': 0}}]}]}],
            sliders=sliders, legend=dict(x=1.05, y=1, xanchor='left', yanchor='top'), margin=dict(r=150)
        )
        fig.show()

if __name__ == "__main__":

    sim = Simulation(
        astar_frame_size=8.0,
        allow_3d=False,
        localized_pbd=False
    )

    sim.add_obstacle(CylinderObstacle(x=50, y=50, radius=25, z_min=0, z_max=20))
    sim.add_obstacle(CylinderObstacle(x=25, y=50, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(CylinderObstacle(x=75, y=50, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(CylinderObstacle(x=50, y=25, radius=15, z_min=0, z_max=20))
    sim.add_obstacle(CylinderObstacle(x=50, y=75, radius=15, z_min=0, z_max=20))

    sim.add_vehicle(Vehicle("HEAD_A", start=[0, 50, 10], end=[100, 50, 10], speed=25, mass_priority=1000, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("HEAD_B", start=[100, 50, 10], end=[0, 50, 10], speed=25, mass_priority=0.001, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("TANK", start=[0, 30, 10], end=[100, 30, 10], speed=30, mass_priority=100000, margin=6, start_time=0))
    sim.add_vehicle(Vehicle("FEATHER", start=[100, 30, 10], end=[0, 30, 10], speed=5, mass_priority=0.0001, margin=3, start_time=0))

    sim.add_vehicle(Vehicle("STACK_A", start=[10, 10, 10], end=[90, 90, 10], speed=12, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("STACK_B", start=[10, 10, 10], end=[90, 10, 10], speed=12, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("STACK_C", start=[10, 10, 10], end=[10, 90, 10], speed=12, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("STACK_D", start=[10, 10, 10], end=[96, 50, 10], speed=12, mass_priority=1, margin=5, start_time=0))

    sim.add_vehicle(Vehicle("NORTH", start=[50, 0, 10], end=[50, 100, 10], speed=18, mass_priority=10, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("SOUTH", start=[50, 100, 10], end=[50, 0, 10], speed=18, mass_priority=10, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("EAST", start=[0, 50, 10], end=[100, 50, 10], speed=18, mass_priority=10, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("WEST", start=[100, 50, 10], end=[0, 50, 10], speed=18, mass_priority=10, margin=4, start_time=0))

    sim.add_vehicle(Vehicle("DIAG_A", start=[0, 0, 10], end=[100, 100, 10], speed=20, mass_priority=5, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("DIAG_B", start=[100, 0, 10], end=[0, 100, 10], speed=20, mass_priority=5, margin=4, start_time=0))

    sim.add_vehicle(Vehicle("CLIMB", start=[0, 20, 5], end=[100, 80, 40], speed=15, mass_priority=10, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("DESCEND", start=[100, 80, 40], end=[0, 20, 5], speed=15, mass_priority=10, margin=4, start_time=0))

    sim.add_vehicle(Vehicle("LOW_CRUISER", start=[0, 60, 2], end=[100, 60, 2], speed=22, mass_priority=10, margin=4, start_time=0))
    sim.add_vehicle(Vehicle("HIGH_CRUISER", start=[100, 60, 18], end=[0, 60, 18], speed=22, mass_priority=10, margin=4, start_time=0))

    sim.add_vehicle(Vehicle("INJECT_A", start=[0, 48, 10], end=[100, 48, 10], speed=28, mass_priority=1, margin=4, start_time=8))
    sim.add_vehicle(Vehicle("INJECT_B", start=[100, 52, 10], end=[0, 52, 10], speed=28, mass_priority=1, margin=4, start_time=8.1))

    sim.add_vehicle(Vehicle("LATE_CROSS", start=[50, 0, 10], end=[50, 100, 10], speed=25, mass_priority=0.01, margin=5, start_time=12))

    # CORRECTED IMPOSSIBLE GOALS / START POSITIONS
    sim.add_vehicle(Vehicle("FAST_X", start=[0, 0, 10], end=[100, 0, 10], speed=250, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("FAST_Y", start=[50, -50, 10], end=[50, 85, 10], speed=250, mass_priority=1, margin=5, start_time=0)) # Moved out of central obs

    sim.add_vehicle(Vehicle("SWAP_A", start=[-20, 0, 10], end=[20, 0, 10], speed=200, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("SWAP_B", start=[20, 0, 10], end=[-20, 0, 10], speed=200, mass_priority=1, margin=5, start_time=0))

    chain_positions = [[0, 150, 10], [24, 150, 10], [48, 150, 10], [72, 150, 10], [96, 150, 10]]
    for i, p in enumerate(chain_positions):
        sim.add_vehicle(Vehicle(f"CHAIN_{i}", start=p, end=[p[0], 250, 10], speed=30, mass_priority=1, margin=12, start_time=0))

    sim.add_vehicle(Vehicle("PBD_A", start=[200, 0, 10], end=[300, 0, 10], speed=10, mass_priority=1, margin=20, start_time=0))
    sim.add_vehicle(Vehicle("PBD_B", start=[201, 0, 10], end=[301, 0, 10], speed=10, mass_priority=1, margin=20, start_time=0))
    sim.add_vehicle(Vehicle("PBD_C", start=[202, 0, 10], end=[302, 0, 10], speed=10, mass_priority=1, margin=20, start_time=0))

    sim.add_vehicle(Vehicle("ZERO_A", start=[350, 50, 10], end=[450, 50, 10], speed=15, mass_priority=1, margin=8, start_time=0))
    sim.add_vehicle(Vehicle("ZERO_B", start=[350, 50, 10], end=[350, 150, 10], speed=15, mass_priority=1, margin=8, start_time=0))

    sim.add_vehicle(Vehicle("INSIDE_OBS", start=[10, 50, 10], end=[150, 50, 10], speed=15, mass_priority=1, margin=5, start_time=0)) # Moved out of central obs
    sim.add_vehicle(Vehicle("BAD_GOAL", start=[0, 50, 10], end=[10, 50, 10], speed=15, mass_priority=1, margin=5, start_time=0)) # Moved out of central obs

    sim.add_vehicle(Vehicle("GRAZER_A", start=[0, 82, 10], end=[100, 82, 10], speed=20, mass_priority=1, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("GRAZER_B", start=[0, 83, 10], end=[100, 83, 10], speed=20, mass_priority=1, margin=5, start_time=0))

    sim.add_vehicle(Vehicle("MASSIVE", start=[0, 300, 10], end=[100, 300, 10], speed=30, mass_priority=1_000_000, margin=5, start_time=0))
    sim.add_vehicle(Vehicle("LIGHT", start=[100, 300, 10], end=[0, 300, 10], speed=30, mass_priority=0.000001, margin=5, start_time=0))

    sim.run()
    sim.plot()
