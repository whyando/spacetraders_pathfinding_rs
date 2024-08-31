use spacetraders_pathfinding_rs::graph::L2Graph;
use spacetraders_pathfinding_rs::systems::System;

fn main() {
    // Load system data
    let charted_systems_json = std::fs::read_to_string("charted_systems.json").unwrap();
    let charted_systems: Vec<System> = serde_json::from_str(&charted_systems_json).unwrap();
    let factions_json = std::fs::read_to_string("factions.json").unwrap();
    let factions: serde_json::Value = serde_json::from_str(&factions_json).unwrap();

    // Create graph
    let graph = L2Graph::from_systems(&charted_systems);

    // Run pathfinding algorithm from src to each faction's headquarters
    let src = "X1-ZU66-93668D";
    println!("Using source = {}\n", src);
    for f in factions.get("data").unwrap().as_array().unwrap() {
        let symbol = f.get("symbol").unwrap().as_str().unwrap();
        let dest = f.get("headquarters").unwrap().as_str().unwrap();
        println!(
            "Running A* pathfinding for destination = {}\t({} HQ)",
            dest, symbol
        );

        let x = graph.astar(src, dest).unwrap() / 60;
        println!("{}\t{}\t{} minutes\n", symbol, dest, x);
    }
}
