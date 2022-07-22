#![deny(missing_debug_implementations)]
use std::{error, fmt, io, time::Duration};

use aes::Aes128;
use cipher::Key;
use dlms_cosem::{hdlc::HdlcDataLinkLayer, Apdu, Data, Dlms};
use reqwest::{Client, Url};
use rppal::gpio::{Gpio, OutputPin};
use serde_json::{Map, Value};
use serialport::{DataBits, Parity, SerialPort, StopBits};
use smart_meter::SmartMeter;

#[derive(Debug)]
pub enum Error {
    Io(io::Error),
    SerialPort(serialport::Error),
    SmartMeter(smart_meter::Error),
    Gpio(rppal::gpio::Error),
    UrlError(url::ParseError),
    Reqwest(reqwest::Error),
    InvalidApduFormat,
    InvalidStatusJson,
    FailedRequest,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Io(err) => err.fmt(f),
            Error::SerialPort(err) => err.fmt(f),
            Error::SmartMeter(err) => err.fmt(f),
            Error::Gpio(err) => err.fmt(f),
            Error::UrlError(err) => err.fmt(f),
            Error::Reqwest(err) => err.fmt(f),
            Error::InvalidApduFormat => write!(f, "invalid apdu format"),
            Error::InvalidStatusJson => write!(f, "invalid JSON status"),
            Error::FailedRequest => write!(f, "failed request"),
        }
    }
}

impl error::Error for Error {
    fn source(&self) -> Option<&(dyn error::Error + 'static)> {
        match self {
            Error::Io(err) => Some(err),
            Error::SerialPort(err) => Some(err),
            Error::SmartMeter(err) => Some(err),
            Error::Gpio(err) => Some(err),
            Error::UrlError(err) => Some(err),
            Error::Reqwest(err) => Some(err),
            _ => None,
        }
    }
}

pub struct Meter {
    smart_meter: SmartMeter<Box<dyn SerialPort>, HdlcDataLinkLayer>,
    buffer: Vec<u8>,
    pin: OutputPin,
}

impl Meter {
    pub fn open(device: &str, key: impl Into<Key<Aes128>>, gpio_pin: u8) -> Result<Self, Error> {
        let reader = serialport::new(device, 115200)
            .parity(Parity::None)
            .data_bits(DataBits::Eight)
            .stop_bits(StopBits::One)
            .timeout(Duration::from_secs(10))
            .open()
            .map_err(Error::SerialPort)?;
        let gpio = Gpio::new().map_err(Error::Gpio)?;
        let pin = gpio.get(gpio_pin).map_err(Error::Gpio)?;
        Ok(Meter {
            smart_meter: SmartMeter::apdu_iter(reader, Dlms::new(key)),
            buffer: Vec::new(),
            pin: pin.into_output(),
        })
    }

    fn read_next(&mut self) -> Result<i32, Error> {
        let apdu = self
            .smart_meter
            .next()
            .unwrap()
            .map_err(Error::SmartMeter)?;

        let data = match &apdu {
            Apdu::DataNotification(data) => data.body(),
            _ => Err(Error::InvalidApduFormat)?,
        };
        let data = match data {
            Data::Structure(data) => data.as_slice(),
            _ => Err(Error::InvalidApduFormat)?,
        };

        let data: Vec<u32> = data
            .iter()
            .filter_map(|d| {
                if let Data::DoubleLongUnsigned(value) = d {
                    Some(*value)
                } else {
                    None
                }
            })
            .collect();

        let available_power = data[5] as i32 - data[4] as i32;
        Ok(available_power)
    }

    pub fn available_power(&mut self) -> Result<i32, Error> {
        self.buffer.clear();
        self.smart_meter
            .reader()
            .clear(serialport::ClearBuffer::Input)
            .map_err(Error::SerialPort)?;
        self.pin.set_high();
        let result = self.read_next();
        self.pin.set_low();
        result
    }
}

impl fmt::Debug for Meter {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Meter")
            .field("buffer", &self.buffer)
            .field("pin", &self.pin)
            .finish()
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ChargingStatus {
    Ready,
    Charging,
    Waiting,
    Finished,
}

impl ChargingStatus {
    fn from_car(car: u8) -> Option<Self> {
        match car {
            1 => Some(ChargingStatus::Ready),
            2 => Some(ChargingStatus::Charging),
            3 => Some(ChargingStatus::Waiting),
            4 => Some(ChargingStatus::Finished),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub struct GoEStatus {
    pub charging_status: ChargingStatus,
    pub ampere: u8,
    pub total_power: u16,
    pub is_charging_allowed: bool,
    pub phases: u8,
}

fn num_from_value(value: &Value) -> Option<u64> {
    match value {
        Value::Number(num) => num.as_u64(),
        Value::String(num_str) => num_str.parse().ok(),
        _ => None,
    }
}

fn get_num(map: &Map<String, Value>, key: &str) -> Option<u64> {
    num_from_value(map.get(key)?)
}

impl GoEStatus {
    fn from_json(json: Value) -> Option<Self> {
        let map = json.as_object()?;
        let car = get_num(map, "car")? as u8;
        let amp = get_num(map, "amp")? as u8;
        let pha = get_num(map, "pha")? as u8;
        let alw = get_num(map, "alw")? as u8;
        let ptot = num_from_value(&map.get("nrg")?.as_array()?[11])? as u16;
        let phases = if (pha & 0xF8) == 0x38 { 3 } else { 1 };
        Some(Self {
            charging_status: ChargingStatus::from_car(car)?,
            ampere: amp,
            total_power: ptot * 10,
            is_charging_allowed: alw > 0,
            phases,
        })
    }
}

#[derive(Debug)]
pub struct GoE {
    url: Url,
    client: Client,
}

impl GoE {
    pub fn open(url: &str) -> Result<Self, Error> {
        Ok(Self {
            url: Url::parse(url).map_err(Error::UrlError)?,
            client: Client::new(),
        })
    }

    pub async fn get_status(&self) -> Result<GoEStatus, Error> {
        let url = self.url.join("status").map_err(Error::UrlError)?;
        let response: Value = self
            .client
            .get(url)
            .send()
            .await
            .map_err(Error::Reqwest)?
            .json()
            .await
            .map_err(Error::Reqwest)?;
        Ok(GoEStatus::from_json(response).ok_or(Error::InvalidStatusJson)?)
    }
    async fn send_message(&self, query: &str) -> Result<(), Error> {
        let mut url = self.url.join("mqtt").map_err(Error::UrlError)?;
        url.set_query(Some(query));
        if self
            .client
            .get(url)
            .send()
            .await
            .map_err(Error::Reqwest)?
            .status()
            .is_success()
        {
            Ok(())
        } else {
            Err(Error::FailedRequest)
        }
    }
    pub async fn set_charging_allowed(&self, is_charging_allowed: bool) -> Result<(), Error> {
        let query = if is_charging_allowed {
            "payload=alw=1"
        } else {
            "payload=alw=0"
        };
        self.send_message(query).await
    }
    pub async fn set_ampere(&self, ampere: u8) -> Result<(), Error> {
        let ampere = ampere.clamp(6, 14);
        let query = format!("payload=amx={}", ampere);
        self.send_message(&query).await
    }
}

#[derive(Debug)]
pub struct RunningAverage<const N: usize> {
    values: [i64; N],
    current_position: usize,
    initialized: bool,
}

impl<const N: usize> RunningAverage<N> {
    pub fn new() -> Self {
        Self {
            values: [0; N],
            current_position: 0,
            initialized: false,
        }
    }

    fn init(&mut self, value: i64) {
        self.values = [value; N];
        self.initialized = true;
    }

    pub fn deinit(&mut self) {
        self.initialized = false;
    }

    pub fn get_average<'a>(&'a self) -> i64 {
        self.values.iter().sum::<i64>() / self.values.len() as i64
    }

    pub fn add(&mut self, value: i64) {
        if self.initialized {
            self.values[self.current_position] = value;
            self.current_position = (self.current_position + 1) % N;
        } else {
            self.init(value)
        }
    }
}
