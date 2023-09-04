#[derive(Serialize, Deserialize)]
struct DataFormat {
    size: u32,
    points: Vec<String>,
}

use futures::StreamExt;
use r2r::robot_localization::srv::FromLL;
use serde::{Deserialize, Serialize};

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let ctx = r2r::Context::create()?;
    let mut node = r2r::Node::create(ctx, "testnode", "")?;

    let client = node.create_client::<FromLL::Service>("/fromLL")?;
    let waiting = node.is_available(&client)?;

    let mut sub_gps =
        node.subscribe::<r2r::std_msgs::msg::String>("gps_waypoints", r2r::QosProfile::default())?;

    tokio::task::spawn(async move {
        println!("waiting for service...");
        waiting.await.unwrap();
        println!("service available.");

        while let Some(msg) = sub_gps.next().await {
            let Ok(data) = serde_json::from_str::<DataFormat>(msg.data.as_str()) else {
                println!("Deserialization failure..");
                continue;
            };

            for i in 0..data.size {
                let ll_point = r2r::geographic_msgs::msg::GeoPoint {
                    latitude: 40.0,
                    longitude: 27.0,
                    altitude: 0.0,
                };
                // TODO: set ll_points attributes
                let req = FromLL::Request { ll_point };
                if let Ok(resp) = client.request(&req).unwrap().await {
                    println!("{:?}", resp.map_point);
                }
                print!("asdf");
            }
        }
    });

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.await?;

    Ok(())
}
