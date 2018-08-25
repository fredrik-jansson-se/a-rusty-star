use std::collections::{HashSet, HashMap};
use std::hash::Hash;
use std::cmp::Eq;
use std::cmp::Ordering;

///
/// Implement this trait for your node type. 
///
/// Call solve to try to find an optimal path between two nodes.
///
/// # Examples
/// ``` 
/// use std::ops::Add;
/// use astar::*;
///
/// #[derive(Debug, Clone, Hash, PartialEq, Eq)]
/// struct Coord {
///    x: i32,
///    y: i32
/// }
///
/// impl Coord {
///     fn new(x: i32, y: i32) -> Coord {
///         Coord {x: x, y: y}
///     }
/// }
///
/// // Helper to simplify neighbor generation
/// impl Add for Coord {
///     type Output = Coord;
///     fn add(self, rhs: Coord) -> Coord {
///         Coord {
///             x: self.x + rhs.x,
///             y: self.y + rhs.y,
///         }
///     }
/// }
///
/// struct MyWorld {
/// }
///
/// impl AStar<Coord> for MyWorld {
///     fn heuristic_cost_estimate(&self, from: &Coord, goal: &Coord) -> f32 {
///         self.distance_between(from, goal)
///     }

///     fn distance_between(&self, from: &Coord, to: &Coord) -> f32 {
///         let dx = (to.x - from.x) as f32;
///         let dy = (to.y - from.y) as f32;
///         (dx * dx + dy * dy).sqrt()
///     }

///     fn neighbors(&self, n: &Coord) -> Vec<Coord> {
///         let mut nbrs = vec!{};
///
///         // Add all neighbors
///         nbrs.push(n.clone() + Coord::new(0, 1));
///         nbrs.push(n.clone() + Coord::new(0, -1));
///         nbrs.push(n.clone() + Coord::new(-1, 0));
///         nbrs.push(n.clone() + Coord::new(1, 0));
///         
///         // Wall for x = 5 and y < 20
///         nbrs.into_iter().filter(|c| c.x != 5 || c.y < 20 ).collect()
///     }
/// }
///
/// let start = Coord::new(0, 0);
/// let goal = Coord::new(10, 10);
/// let my_world = MyWorld {};
/// let solution = my_world.solve(&start, &goal);
/// assert_eq!(true, solution.is_some());
/// ```
pub trait AStar<Node: Clone + Eq + Hash> {
    /// This function should return the heuristic distance between
    /// a node and the goal.
    fn heuristic_cost_estimate(&self, from: &Node, goal: &Node) -> f32;

    /// Returns all valid neighbors of the given node.
    fn neighbors(&self, n: &Node) -> Vec<Node>;

    /// Return the distance between two nodes.
    fn distance_between(&self, a: &Node, b: &Node) -> f32;

    ///
    /// Tries to try to solve the shortest path between start and goal.
    ///
    /// If a path is found, the nodes are returned as Some(nodes) if no path is
    /// found, None is returned.
    ///
    ///
    fn solve(&self, start: &Node, goal: &Node) -> Option<Vec<Node>> {
        let mut closed_set = HashSet::new();

        let mut open_set = HashSet::new();
        open_set.insert(start.clone());

        let mut came_from = HashMap::new();

        let mut g_score = HashMap::new();

        g_score.insert(start.clone(), 0.0);

        let mut f_score = HashMap::new();
        f_score.insert(start.clone(), self.heuristic_cost_estimate(start, goal));

        while !open_set.is_empty() {
            let current = open_set.iter().min_by(|c1,c2| {
                let f1 : f32 = get_score(&f_score, c1);
                let f2 : f32 = get_score(&f_score, c2);
                f1.partial_cmp(&f2).unwrap_or(Ordering::Equal)
            }).unwrap().clone();

            if current == *goal {
                return Some(reconstruct_path(&came_from, &current));
            }

            open_set.remove(&current);
            closed_set.insert(current.clone());

            for n in self.neighbors(&current).iter() {
                if closed_set.contains(n) {
                    continue;
                }

                if !open_set.contains(n) {
                    open_set.insert(n.clone());
                }

                let current_g_score = g_score.get(&current).unwrap().clone();

                let nbr_g_score = get_score(&g_score, n);

                let tentative_g_score = current_g_score + self.distance_between(&current, n); 
                if tentative_g_score >= nbr_g_score {
                    continue;
                }
                came_from.insert(n.clone(), current.clone());
                g_score.insert(n.clone(), tentative_g_score);
                f_score.insert(n.clone(), tentative_g_score + self.heuristic_cost_estimate(n, goal));
            }
        }
        None
    }
}

fn reconstruct_path<Node>(came_from: &HashMap<Node, Node>, current: &Node) -> Vec<Node> 
    where Node: Eq+Hash+Clone
{
    let mut total_path = vec!{};
    
    let mut c = current;

    loop {
        match came_from.get(c) {
            None => break,
            Some(new_c) => {
                total_path.push(c.clone());
                c = new_c;
            }
        }
    }

    total_path.reverse();
    total_path
}

fn get_score<N: Hash+Eq>(m: &HashMap<N, f32>, k: &N) -> f32 
{
    match m.get(k) {
        Some(v) => *v,
        None => std::f32::MAX
    }
}



#[cfg(test)]
mod tests {
    use std::ops::Add;
    use super::*;

    #[derive(Debug, Clone, Hash, PartialEq, Eq)]
    struct Coord {
        pub x: isize,
        pub y: isize,
    }

    impl Coord {
        fn new(x: isize, y:isize) -> Coord {
            Coord {
                x: x,
                y: y,
            }
        }
    }

    impl Add for Coord {
        type Output = Coord;

        fn add(self, rhs: Coord) -> Coord {
            Coord {
                x: self.x + rhs.x,
                y: self.y + rhs.y,
            }
        }
    }

    struct CoordAStar {
    }

    impl AStar<Coord> for CoordAStar {
        fn heuristic_cost_estimate(&self, from: &Coord, goal: &Coord) -> f32 {
            self.distance_between(from, goal)
        }

        fn distance_between(&self, from: &Coord, to: &Coord) -> f32 {
            let dx = (to.x - from.x) as f32;
            let dy = (to.y - from.y) as f32;
            (dx * dx + dy * dy).sqrt()
        }

        fn neighbors(&self, n: &Coord) -> Vec<Coord> {
            let mut r = vec!{};

            r.push(n.clone() + Coord::new(0, 1));
            r.push(n.clone() + Coord::new(0, -1));
            r.push(n.clone() + Coord::new(-1, 0));
            r.push(n.clone() + Coord::new(1, 0));

            r
        }
    }


    struct CoordAStar2 {
    }

    impl AStar<Coord> for CoordAStar2 {
        fn heuristic_cost_estimate(&self, from: &Coord, goal: &Coord) -> f32 {
            self.distance_between(from, goal)
        }

        fn distance_between(&self, from: &Coord, to: &Coord) -> f32 {
            let dx = (to.x - from.x) as f32;
            let dy = (to.y - from.y) as f32;
            (dx * dx + dy * dy).sqrt()
        }

        fn neighbors(&self, n: &Coord) -> Vec<Coord> {
            let mut r : Vec<Coord> = vec!{};

            r.push(n.clone() + Coord::new(0, 1));
            r.push(n.clone() + Coord::new(0, -1));
            r.push(n.clone() + Coord::new(-1, 0));
            r.push(n.clone() + Coord::new(1, 0));

            r.into_iter().filter(|c| *c != Coord::new(1,0) && *c != Coord::new(1,1) && c.y >= 0).collect()
        }
    }

    #[test]
    fn simple() {
        let start = Coord::new(0,0);
        let goal = Coord::new(3,3);
        let astar = CoordAStar{};

        let res = astar.solve(&start, &goal);
        assert_eq!(res.is_some(), true);
        assert_eq!(res.unwrap().len(), 6);
    }

    #[test]
    fn with_wall() {
        //
        //  |
        // S|G
        // -------
        let start = Coord::new(0,0);
        let goal = Coord::new(2,0);
        let astar = CoordAStar2{};

        let res = astar.solve(&start, &goal);
        println!("res: {:?}", res);
        assert_eq!(res.is_some(), true);
        assert_eq!(res.unwrap().len(), 6);
    }
}
