use crate::math::Vector2;

#[derive(Debug, Clone, Copy)]
pub struct Obstacle {
    pub center: Vector2,
    pub radius: f64,
}

impl Obstacle {
    pub fn new(center: Vector2, radius: f64) -> Self {
        Self { center, radius }
    }

    pub fn distance_from_edge(self, point: Vector2) -> f64 {
        point.distance(self.center) - self.radius
    }
}

#[derive(Debug, Clone)]
pub struct ObstacleField {
    obstacles: Vec<Obstacle>,
}

impl ObstacleField {
    pub fn new(obstacles: Vec<Obstacle>) -> Self {
        Self { obstacles }
    }

    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    pub fn nearest_threat(&self, position: Vector2, safety_distance: f64) -> Option<Obstacle> {
        self.obstacles
            .iter()
            .copied()
            .filter(|obstacle| obstacle.distance_from_edge(position) <= safety_distance)
            .min_by(|a, b| {
                a.distance_from_edge(position)
                    .partial_cmp(&b.distance_from_edge(position))
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    pub fn avoidance_direction(&self, position: Vector2, safety_distance: f64) -> Option<Vector2> {
        self.nearest_threat(position, safety_distance)
            .map(|obstacle| (position - obstacle.center).normalize())
    }
}

#[derive(Debug, Clone)]
pub struct WaypointNavigator {
    waypoints: Vec<Vector2>,
    active: usize,
    arrival_radius: f64,
}

impl WaypointNavigator {
    pub fn new(waypoints: Vec<Vector2>, arrival_radius: f64) -> Self {
        Self {
            waypoints,
            active: 0,
            arrival_radius,
        }
    }

    pub fn active_index(&self) -> Option<usize> {
        (self.active < self.waypoints.len()).then_some(self.active)
    }

    pub fn active_waypoint(&self) -> Option<Vector2> {
        self.waypoints.get(self.active).copied()
    }

    pub fn is_finished(&self) -> bool {
        self.active >= self.waypoints.len()
    }

    pub fn direction_to_active(&self, position: Vector2) -> Option<Vector2> {
        self.active_waypoint()
            .map(|waypoint| (waypoint - position).normalize())
    }

    pub fn update(&mut self, position: Vector2) -> bool {
        if let Some(waypoint) = self.active_waypoint() {
            if position.distance(waypoint) <= self.arrival_radius {
                self.active += 1;
            }
        }
        self.is_finished()
    }
}

#[cfg(test)]
mod tests {
    use super::{Obstacle, ObstacleField, WaypointNavigator};
    use crate::math::Vector2;

    #[test]
    fn waypoint_completion_advances_until_done() {
        let mut nav = WaypointNavigator::new(vec![Vector2::new(1.0, 0.0)], 0.5);
        assert!(!nav.update(Vector2::ZERO));
        assert!(nav.update(Vector2::new(1.1, 0.0)));
        assert!(nav.is_finished());
    }

    #[test]
    fn obstacle_detection_finds_nearby_circle() {
        let field = ObstacleField::new(vec![Obstacle::new(Vector2::new(1.0, 0.0), 0.25)]);
        assert!(field.nearest_threat(Vector2::new(0.9, 0.0), 0.5).is_some());
    }
}
