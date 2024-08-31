use spacetraders_pathfinding_rs::graph::L2Graph;
use spacetraders_pathfinding_rs::systems::{System, Waypoint};

fn main() {
    let test = std::fs::read_to_string("space_traders_tests/pathing/simple.json").unwrap();
    let test: test_models::TestSuite = serde_json::from_str(&test).unwrap();

    let mut systems = vec![];
    for s in test.systems {
        let waypoints = s
            .waypoints
            .iter()
            .map(|w| Waypoint {
                symbol: w.symbol.clone(),
                x: w.x,
                y: w.y,
                waypoint_type: "PLANET".to_string(),
                traits: None,
            })
            .collect();
        let system = System {
            symbol: s.symbol,
            system_type: "STAR".to_string(),
            x: 0,
            y: 0,
            waypoints,
        };
        systems.push(system);
    }
    let _graph = L2Graph::from_systems(&systems);
    todo!("Finish applying pathfinding algorithm to test cases");
}

pub mod test_models {
    use serde::{Deserialize, Serialize};

    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct Ship {
        pub speed: i32,
        pub fuel_capacity: i32,
        #[serde(rename = "fuel")]
        pub intial_fuel: i32,
    }
    #[derive(Serialize, Deserialize, Debug)]
    pub struct System {
        pub symbol: String,
        // (may be needed for tests involving jumpgates)
        // system_type: String,
        // (needed for tests involving inter system travel)
        // x: i32,
        // y: i32,
        pub waypoints: Vec<Waypoint>,
    }
    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct Waypoint {
        pub symbol: String,
        pub x: i32,
        pub y: i32,
        // default false
        #[serde(default = "bool::default")]
        pub sells_fuel: bool,
    }
    #[derive(Serialize, Deserialize, Debug)]
    pub struct TestSuite {
        pub ship: Ship,
        pub systems: Vec<System>,
        pub tests: Vec<Test>,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Test {
        pub start: String,
        pub end: String,
        pub expect: Expected,
    }

    #[derive(Serialize, Deserialize, Debug)]
    #[serde(rename_all = "camelCase")]
    pub struct Expected {
        pub route: Vec<Action>,
        pub fuel_used: i32,
        pub time: i32,
    }

    #[derive(Serialize, Deserialize, Debug)]
    pub struct Action {
        pub action: String,
        pub start: String,
        pub end: String,
    }
}
