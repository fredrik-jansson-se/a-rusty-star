use std::collections::{HashSet, HashMap};
use std::hash::Hash;
use std::cmp::Eq;
use std::cmp::Ordering;

pub trait AStar<Node: Clone + Eq + Hash> {
    fn heuristic_cost_estimate(&self, from: &Node, to: &Node) -> f64;
    fn neighbors(&self, n: &Node) -> Vec<Node>;
    fn distance_between(&self, a: &Node, b: &Node) -> f64;

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
                let f1 : f64 = get_score(&f_score, c1);
                let f2 : f64 = get_score(&f_score, c2);
                if f1 < f2 {
                    Ordering::Less
                }
                else if f1 == f2 {
                    Ordering::Equal
                }
                else {
                    Ordering::Greater
                }
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

fn get_score<N: Hash+Eq>(m: &HashMap<N, f64>, k: &N) -> f64 
{
    match m.get(k) {
        Some(v) => *v,
        None => std::f64::MAX
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
        fn heuristic_cost_estimate(&self, from: &Coord, to: &Coord) -> f64 {
            self.distance_between(from, to)
        }

        fn distance_between(&self, from: &Coord, to: &Coord) -> f64 {
            let dx = (to.x - from.x) as f64;
            let dy = (to.y - from.y) as f64;
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
        fn heuristic_cost_estimate(&self, from: &Coord, to: &Coord) -> f64 {
            self.distance_between(from, to)
        }

        fn distance_between(&self, from: &Coord, to: &Coord) -> f64 {
            let dx = (to.x - from.x) as f64;
            let dy = (to.y - from.y) as f64;
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
