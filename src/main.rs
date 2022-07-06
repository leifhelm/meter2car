use std::{env, error::Error, thread::sleep, time::Duration};

use hex::FromHex;
use meter2car::{ChargingStatus, GoE, Meter, RunningAverage};

const TURN_ON_THRESHOLD: i64 = 1500;
const TURN_OFF_THRESHOLD: i64 = 1200;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    let url = env::args()
        .nth(1)
        .expect("Expected Go-e url as first argument");
    let key = env::var("METER2CAR_KEY")?;
    let key = <[u8; 16]>::from_hex(key).expect("Invalid key format");

    let go_e = GoE::open(&url)?;
    let mut meter = Meter::open("/dev/serial0", key, 2)?;

    let mut turn_off_counter = 0;
    let mut power_for_car_runnning_average = RunningAverage::<5>::new();
    let mut available_power_running_average = RunningAverage::<5>::new();

    loop {
        let available_power = meter.available_power()?;
        println!("available power: {}", available_power);
        let status = go_e.get_status().await?;
        println!("{:#?}", status);

        if status.is_charging_allowed {
            if status.charging_status == ChargingStatus::Charging {
                let power_for_car = available_power + status.total_power as i32;
                power_for_car_runnning_average.add(power_for_car as i64);
                let average_power_for_car = power_for_car_runnning_average.get_average();
                println!(
                    "average power for car: {}, turn off counter: {}",
                    average_power_for_car, turn_off_counter
                );
                println!(
                    "TURN_OFF_THRESHOLD: {}",
                    (TURN_OFF_THRESHOLD * status.phases as i64)
                );
                let desired_ampere = average_power_for_car / (230 * status.phases as i64);
                if turn_off_counter >= 4 {
                    println!(
                        "{} < {}: {}",
                        average_power_for_car,
                        (TURN_OFF_THRESHOLD * status.phases as i64),
                        average_power_for_car < (TURN_OFF_THRESHOLD * status.phases as i64)
                    );
                    if average_power_for_car < (TURN_OFF_THRESHOLD * status.phases as i64) {
                        println!("Disable charging");
                        go_e.set_charging_allowed(false).await?;
                        power_for_car_runnning_average.deinit();
                    }
                    turn_off_counter = 0;
                }
                if desired_ampere != status.ampere as i64 {
                    if desired_ampere < 6 {
                        println!("desired ampere < 6");
                    } else if desired_ampere > 16 {
                        println!("desired ampere > 16");
                    }
                    go_e.set_ampere(desired_ampere as u8).await?;
                }
            }
        } else {
            available_power_running_average.add(available_power as i64);
            let average_available_power = available_power_running_average.get_average();
            println!("average available power: {}", average_available_power);
            if status.charging_status == ChargingStatus::Finished
                || status.charging_status == ChargingStatus::Waiting
            {
                if average_available_power > (TURN_ON_THRESHOLD * status.phases as i64) {
                    let ampere = average_available_power / (230 * status.phases as i64);
                    println!("Enable charging at {}A", ampere);
                    go_e.set_ampere(ampere as u8).await?;
                    sleep(Duration::from_secs(5));
                    go_e.set_charging_allowed(true).await?;
                }
            }
        }
        // Wait
        sleep(Duration::from_secs(60));
        turn_off_counter += 1;
    }
}
