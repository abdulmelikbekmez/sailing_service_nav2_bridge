#[derive(Serialize, Deserialize)]
struct DataFormat {
    size: i32,
    points: Vec<f32>,
}

use futures::StreamExt;
use r2r::{geometry_msgs::msg::PoseStamped, robot_localization::srv::FromLL};
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
            println!("message received from publisher {:?}", msg.data);
            let Ok(data) = serde_json::from_str::<DataFormat>(msg.data.as_str()) else {
                println!("Deserialization failure..");
                continue;
            };

            let mut goal_poses = vec![];

            for i in 0..data.size {
                let mut ll_point = r2r::geographic_msgs::msg::GeoPoint::default();
                ll_point.latitude = data.points[(i * 2) as usize] as f64;
                ll_point.longitude = data.points[(i * 2 + 1) as usize] as f64;
                let req = FromLL::Request { ll_point };
                println!("request => {:?}", req);
                if let Ok(resp) = client.request(&req).unwrap().await {
                    println!("{:?}", resp.map_point);

                    let mut pose = PoseStamped::default();
                    // pose.header.stamp
                    pose.pose.position.x = resp.map_point.x;
                    pose.pose.position.y = resp.map_point.y;
                    goal_poses.push(pose);
                }
            }

            println!("goal poses => {:?}", goal_poses)
        }
    });

    let handle = tokio::task::spawn_blocking(move || loop {
        node.spin_once(std::time::Duration::from_millis(100));
    });

    handle.await?;

    Ok(())
}
