// Imports //
use std::io::{self, Write, Read, Error, ErrorKind};
use clap::Parser;
use serialport::{SerialPort, TTYPort};
use hyprland::shared::{HyprData, HyprDataVec};
use std::process::Command;

// Console arguments //
#[derive(Parser, Debug)]
pub struct Args {
    #[arg(short, long, default_value_t = String::from("/dev/ttyACM0"), help = "Arduino /dev/ path")]
    device: String,

    #[arg(short, long, default_value_t = 9600, help = "Serial port baud rate")]
    baud_rate: u32,

    #[arg(long, default_value_t = String::from("HDMI-A-1"))]
    side_monitor: String,

    #[arg(long, default_value_t = String::from("DP-1"))]
    main_monitor: String,

    #[arg(short, long, default_value_t = 65, help = "Relative angle at which the monitor switches orientations")]
    threshold: u8,

    #[arg(long, default_value_t = 50, help = "Millisecond delay between commands")]
    delay: u64,

    #[arg(index = 1, num_args = 0.., help =
"Supported commands:
\r   listen      - Listen for arduino commands                                  [alt: l*]
\r   query       - Send an ENQ to check arduino's running status                [alt: ENQ]
\r   recalibrate - Make the arduino to recalibrate its IMU                      [alt: DC2]
\r   start       - Stop the arduino                                             [alt: DC1]
\r   stop        - Start the arduino                                            [alt: DC3]
\r   stop-all    - Stop the arduino and make it send EOT to all listeners       [alt: DC4]
\r   test-comms  - Send a SYN to test comms                                     [alt: SYN]
\r   ble-command - Process the next command in BLE mode                         [alt: DLE]
\r   text-start  - Send STX and interpret commands until ETX as literal strings [alt: STX]
\r   text-end    - End of text                                                  [alt: ETX]")]
    commands: Vec<String>
}

// Constants //
const SYN: u8 = 0x16;
const ACK: u8 = 0x06;
const NAK: u8 = 0x15;
const ENQ: u8 = 0x05;
const DC1: u8 = 0x11;
const DC2: u8 = 0x12;
const DC3: u8 = 0x13;
const DC4: u8 = 0x14;
const EOT: u8 = 0x04;
const SUB: u8 = 0x1A;
const DLE: u8 = 0x10;
const STX: u8 = 0x02;
const ETX: u8 = 0x03;

// Main function //
fn main() -> Result<(), Error> {
    let args = Args::parse();
    
    print!("Opening serial port at {}...", args.device); io::stdout().flush()?;
    let mut serial = serialport::new(args.device, 9600).open_native()?;
    println!("done");
    
    _=serial.set_exclusive(false);

    let mut text_mode = false;

    for command in args.commands.into_iter() {
        if text_mode {
            send_bytes(&mut serial, (command.as_bytes(), &command))?;
            if command == "text-end" || command == "ETX" {
                text_mode = false;
                send_signal(&mut serial, (ETX, "ETX"))?;
            }
        } else {
            if command.starts_with('l') {
                loop {
                    if serial.bytes_to_read()? > 0 {
                        let mut buf = [0u8; 1];
                        serial.read_exact(&mut buf)?;
                        handle_signal(&mut serial, buf[0])?;
                    }
                }
            }

            send_signal(&mut serial, match command.as_str() {
                "query"       | "ENQ" => (ENQ, "ENQ"),
                "recalibrate" | "DC2" => (DC2, "DC2"),
                "start"       | "DC1" => (DC1, "DC1"),
                "stop"        | "DC3" => (DC3, "DC3"),
                "stop-all"    | "DC4" => (DC4, "DC4"),
                "test-comms"  | "SYN" => (SYN, "SYN"),
                "ble-command" | "DLE" => (DLE, "DLE"),
                "text-start"  | "STX" => {text_mode = true;  (STX, "STX")},
                 _ => panic!("Error: Unknown command \"{}\".", command)
            })?;

        }

        std::thread::sleep(std::time::Duration::from_millis(args.delay));
    }
    Ok(())
}

// Misc. functions //
fn handle_signal(serial: &mut TTYPort, signal: u8) -> Result<(), Error> {
    match signal {
        ACK => println!("ACK received."),
        NAK => println!("NAK received."),
        SUB => println!("SUB received."),
        ENQ => {
            print!("ENQ received.\nGetting monitor data..."); io::stdout().flush()?;

            let args = Args::parse();
            if match hyprland::data::Monitors::get() {
                Ok(m) => if let Some(monitor) = m.to_vec().into_iter().find(|m| m.name == args.side_monitor) {
                    println!("done");
                    let code = (monitor.transform as u8)%4;

                    send_signal(serial, (ACK, "ACK"))?;
                    send_signal(serial, (code, format!("monitor orientation {:#04x}", code)))?;
                    send_signal(serial, (args.threshold, format!("angle threshold {:#04x}", args.threshold)))?;

                    false
                } else {println!("error, monitor {} not found.", args.side_monitor); true}
                Err(e) => {println!("error: {}", e.to_string()); true}
            } {send_signal(serial, (NAK, "NAK"))?;}
        }
        SYN => {
            println!("SYN received.");
            send_signal(serial, (ACK, "ACK"))?;
        }
        EOT => {
            println!("EOT receieved.");
            send_signal(serial, (ACK, "ACK"))?;
            std::process::exit(0);
        }
        s if (DC1..=DC4).contains(&s) => {
            let code = s-DC1;
            let args = Args::parse();
            println!("DC{} received.\nRotating monitor...", code+1);

            if let Err(e) = hyprland::keyword::Keyword::set("monitor", format!("{},transform,{}", args.side_monitor, code)) {
                return Err(Error::new(ErrorKind::Other, e.to_string()))
            }
            if let Err(e) = hyprland::keyword::Keyword::set("monitor", format!("{},1920x1080@144,{},1", args.main_monitor, if code%2==0 {"1920x0"} else {"1080x400"})) {
                return Err(Error::new(ErrorKind::Other, e.to_string()))
            }
            Command::new("kitty").args(["@","--to","tcp:localhost:65065","goto-layout","-m","all",if code%2==0 {"grid"} else {"vertical"}]).output()?;

            println!("done");

            send_signal(serial, (ACK, "ACK"))?;
        }
        _   => {
            println!("error, received unrecognized code {signal}.");
            send_signal(serial, (SUB, "SUB"))?;
        }
    }

    Ok(())
}

fn send_signal<S: AsRef<str> + std::fmt::Display>(serial: &mut TTYPort, payload: (u8, S)) -> Result<(), Error> {
    send_bytes(serial, (&[payload.0], payload.1))
}

fn send_bytes<S: AsRef<str> + std::fmt::Display>(serial: &mut TTYPort, payload: (&[u8], S)) -> Result<(), Error> {
    print!("Sending {}...", payload.1); io::stdout().flush()?;

    if let Err(e) = || -> Result<(), Error> {
        serial.write(payload.0)?;
        serial.flush()?;
        println!("done");
        Ok(())
    }() {println!("error, couldn't send: {}", e.to_string());}

    Ok(())
}
