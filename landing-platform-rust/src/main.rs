extern crate libmodbus_rs;
extern crate libudev_sys as ffi;
use std::env;
use std::path::Path;
use std::time::Duration;
use std::io::prelude::*;
use std::fs::File;
use serialport::posix::TTYPort;
use socketcan::{CANSocket, CANSocketOpenError, CANFrame};
use libmodbus_rs::{Modbus, ModbusRTU, SerialMode, ModbusClient};
use serialport::open;

// use mosq::Mosquitto;
// use std::{thread, time};


struct BMSDPack {
    header: u8,
    controller_communications_data: u8,
    command_code: u8,
    data: u8,
    crc: u8
}

impl BMSDPack {
    fn calc_check_sum_bmsd(&mut self) -> &mut BMSDPack{
        let buffer_bmsd:[u8; 5]= [
            self.header,
            self.controller_communications_data,
            self.command_code,
            self.data,
            self.crc
        ];
        let mut seed: u8 = 0x00;
        for j in 1..4 {
            let mut in_data = buffer_bmsd[j];
            for _i in 0..8 {
                let temp = (seed ^ in_data) & 0x01;
                if temp == 0{
		    
                    seed = seed >> 1;
                }else{
                    seed ^= 0x18;
                    seed >>= 1;
                    seed |= 0x80;
                }
                in_data >>= 1;
            }
        }
        self.crc = seed;
        self
    }

    fn print_message_bmsd(&mut self){
        match self.header{
            0xA2 => println!("Импульсы датчика Холла: {}", self.data),
            0xA3 => println!("Скорость: {}", self.data),
            0xA4 => println!("Максимальная скорость: {}", self.data),
            0xA5 => println!("Ускорение: {}", self.data),
            0xA6 => println!("Торможение: {}", self.data),
            0xA7 => println!("Направление: {}", self.data),
            _ => println!("Параметр отсуствует"),
        }
    }

    fn send_msg_bmsd(&mut self, number: u8) -> &mut BMSDPack{
        self.controller_communications_data = number;
        BMSDPack::calc_check_sum_bmsd(self);
	self
    }
    /// Функция выполненяет команды для подъемного механизма
    ///
    /// ```
    /// command:
    ///   up - вверх
    ///   down - вниз
    ///   stop - остановить
    /// ```
    /// ```
    /// number:
    ///   1,2,3,4 - номер подъемного механизма
    ///   5 - все подъемные механизы
    /// ```
    fn action_bmsd(command: &str, number: u8){
	match command {
	    "up" => {
		let mut command_pack_bmsd: [BMSDPack; 5] =
		    [
			BMSDPack {
			    header: 0xE6,
			    controller_communications_data: 0x00,
			    command_code: 0xA3,
			    data: 0x30,
			    crc: 0x00
			},
			BMSDPack {
			    header: 0xE6,
			    controller_communications_data: 0x00,
			    command_code: 0xA5,
			    data: 0x24,
			    crc: 0x00
			},
			BMSDPack {
			    header: 0xE6,
			    controller_communications_data: 0x00,
			    command_code: 0xA6,
			    data: 0x24,
			    crc: 0x00
			},
			BMSDPack {
			    header: 0xE6,
			    controller_communications_data: 0x00,
			    command_code: 0xA7,
			    data: 0x01,
			    crc: 0x00
			},
			BMSDPack {
			    header: 0xE6,
			    controller_communications_data: 0x00,
			    command_code: 0x51,
			    data: 0x00,
			    crc: 0x00
			}
		    ];
            match number {	
                1 => {
                    for i in 1..6{
			BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
		    }
                }
                2 => {
                    for i in 1..6{
			BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
		    }
                } 
                3 => {
                    for i in 1..6{
			BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
		    }
                } 
                4 => {
                    for i in 1..6{
			BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
		    }
                }   
                5 => {
		        for j in 0..4{
			    for i in 0..5{
				BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			    }
		        }
                }  
                _ => println!("Номер лифта отсутствует"),
            }
	    },
	    "down" =>  {
		let mut command_pack_bmsd: [BMSDPack; 5] = [
                    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA3,
			data: 0x30,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA5,
			data: 0x24,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA6,
			data: 0x24,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA7,
			data: 0x00,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0x51,
			data: 0x00,
			crc: 0x00
                    }
		];
		match number {	
                    1 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    }
                    2 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    } 
                    3 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    } 
                    4 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    }   
                    5 => {
			for i in 0..4{
			    for i in 0..5{
				BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			    }
		        }
                    }  
                    _ => println!("Номер лифта отсутствует"),
		}
	    },
	    "stop" => {
		let mut command_pack_bmsd: [BMSDPack; 5] = [
                    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA3,
			data: 0x00,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA5,
			data: 0x24,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA6,
			data: 0x24,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0xA7,
			data: 0x00,
			crc: 0x00
                    },
		    BMSDPack {
			header: 0xE6,
			controller_communications_data: 0x00,
			command_code: 0x51,
			data: 0x00,
			crc: 0x00
                    }
		];
		match number {	
                    1 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    }
                    2 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    } 
                    3 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    } 
                    4 => {
			for i in 0..5{
			    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], number);
			}
                    }   
                    5 => {
			for i in 0..4{
			    for j in 0..5{
				command_pack_bmsd[j].controller_communications_data = i;
				BMSDPack::calc_check_sum_bmsd(&mut command_pack_bmsd[j]);
			    }
			}
                    }  
                    _ => println!("Номер лифта отсутствует"),
		}
	    },
	    _ => println!("Команда отсутствует"),
	}
    }
}


struct AWDPack {
    address: u8,
    cmdcode: u8,
    param1: u8,
    param0: u8,
    data1: u8,
    data0: u8,
    status: u8,
    cs: i8
}

impl AWDPack{
    fn calc_check_sum_awd(&mut self)->&mut AWDPack {
        self.cs = 0x00 - (self.address as i8 + self.cmdcode as i8 +self.param1 as i8 +self.param0 as i8 + self.data1 as i8 + self.data0 as i8 +self.status as i8);
        return self
    }


    fn print_message_awd(&mut self){
        match self.address{
            0xA2 => println!("Импульсы датчика Холла: {}", self.data1),
            0xA3 => println!("Скорость: {}", self.data1),
            0xA4 => println!("Максимальная скорость: {}", self.data1),
            0xA5 => println!("Ускорение: {}", self.data1),
            0xA6 => println!("Торможение: {}", self.data1),
            0xA7 => println!("Направление: {}", self.data1),
            _ => println!("Параметр отсуствует"),
        }
    }

    /// Функция выполненяет команды для центрующего механизма
    ///
    /// ```
    /// command:
    ///   forward - вперед
    ///   back - назад
    ///   stop - остановить
    /// ```
    /// ```
    /// number:
    ///   11,12,13,14 - номера центрующих механизмов
    ///   21,22,23,24 - номера центрующих механизмов
    ///   00 - все центрующие механизы
    /// ```

    fn action_awd(command: &str, number: u8){
	match command {
	    "forward" =>{
		
	    }
	    "back" =>{
		
	    }
	    "stop" => {
		
	    }
	    _ => println!("Команда отсутствует")
	}
    }
}

/// Функция открывает порт для ПЧВ
fn open_oven<'a>(modbus_platform: &'a mut Modbus, number_door: u8)-> &'a mut Modbus{
    assert!(modbus_platform.set_slave(number_door).is_ok());
    assert!(modbus_platform.set_debug(true).is_ok());
    // assert!(modbus_platform.rtu_set_serial_mode(SerialMode::RtuRS485).is_ok());
    
    match modbus_platform.connect(){
        Ok(_) => {  }
        Err(e) => println!("Error: {}", e),
    };
    
    modbus_platform
}

/// Функция закрывает порт для ПЧВ 
fn close_oven<'a>(modbus_platform: &'a mut Modbus){
    modbus_platform.close();
    modbus_platform.free();
}
    
/// Функция выполненяет команды для подъемного механизма верхней крышки
///
/// ```
/// command:
/// open - открыть
/// close - закрыть
/// stop - остановить
/// status - состояние привода
/// count - текущие значение привода
/// ```
/// ```
/// number_motor:
///   7,8 - номер подъемного механизма
/// ```
fn send_msg_oven<'a>(modbus_platform: &'a  mut Modbus, command_door: &'a str, number_door: &'a u8){
    match command_door{
	"open" => {
	    match number_door{
		7 => {
		    assert!(modbus_platform.write_register(49999, 33916).is_ok());
		    assert!(modbus_platform.write_register(50009, 2000).is_ok());
		}
		
		8 => {
		    assert!(modbus_platform.write_register(49999, 33916).is_ok());
		    assert!(modbus_platform.write_register(50009, 2000).is_ok());
		}
		_ => println!("Не правильный адрес")
	    }
	}
	"close" => {
	    match number_door{
		7 => {
		    assert!(modbus_platform.write_register(49999, 1148).is_ok());
		    assert!(modbus_platform.write_register(50009, 3000).is_ok());
		}
		    
		8 => {
		    assert!(modbus_platform.write_register(49999, 1148).is_ok());
		    assert!(modbus_platform.write_register(50009, 2000).is_ok());
		}
		_ => println!("Не правильный адрес")
	    }
	}
	"stop" => {
		match number_door{
		    7 => {
			assert!(modbus_platform.write_register(49999, 0).is_ok());
			assert!(modbus_platform.write_register(50009, 0).is_ok());
		    }
		    
		    8 => {
			assert!(modbus_platform.write_register(49999, 0).is_ok());
			assert!(modbus_platform.write_register(50009, 0).is_ok());
		    }
		    _ => println!("Не правильный адрес")
		}
	}
	_ => println!("Не правильная комманда")
    }    
}

struct GEFRANPack{
    x_result: i16,
    y_result: i16,
    byte_1: u16,
    byte_2: u16
}

/// Функция получает угл наклона в байтах и преобразует в градусы
///
/// ```
/// can_port - порт CAN шины 
/// ```
impl GEFRANPack{
    fn give_position_canbus<'a>(can_port: &'a str) -> Option <GEFRANPack>{
	let socket_can = CANSocket::open(can_port).ok().expect("Error: Can't open CAN bus port");
	let timeout_can = Duration::new(5,0);
	
	let mut position_gefran = GEFRANPack{
	    x_result: 0,
	    y_result: 0,
	    byte_1: 0,
	    byte_2: 0
	};
	
	//socket_can.set_nonblocking(true).unwrap();
	socket_can.set_read_timeout(timeout_can).unwrap();
	
	match socket_can.read_frame() {
	    Ok(frame) => {
		let buffer: &[u8] = frame.data();
		position_gefran.byte_1 = ((buffer[0] as u16) << 8) + buffer[1] as u16;
		position_gefran.byte_2 = ((buffer[2] as u16) << 8) + buffer[3] as u16;

			if (position_gefran.byte_1 > 32767){
				position_gefran.x_result = (0-((65535 - (position_gefran.byte_1)) as i16 /100));
			}
			else {
				position_gefran.x_result = (position_gefran.byte_1/100) as i16;
			}

			if (position_gefran.byte_2 > 32767){
				position_gefran.y_result = (0-((65535 - (position_gefran.byte_2)) as i16 /100));
			}
			else {
				position_gefran.y_result = ((position_gefran.byte_2/100) as i16) ;
			}
			return Some(position_gefran)
	    },
	    Err(e) => panic!("Can't read frame from CAN socket {:?}", e),
    }
        None
    }
}

#[test]
fn test_calc_check_sum_bmsd(){
    let mut test_command: [BMSDPack; 3] = [
        BMSDPack {
            header: 0xE6,
            controller_communications_data: 0xFF,
            command_code: 0xA0,
            data: 0x01,
            crc: 0x00
        },
        BMSDPack {
            header: 0xE6,
            controller_communications_data: 0x00,
            command_code: 0xA2,
            data: 0x02,
            crc: 0x00
        },
        BMSDPack {
            header: 0xE6,
            controller_communications_data: 0x00,
            command_code: 0xA4,
            data: 0x43,
            crc: 0x00
        }
    ];
    let test:&mut BMSDPack = BMSDPack::calc_check_sum_bmsd(&mut test_command[0]);
    assert_eq!(0x62, test.crc);
    let test:&mut BMSDPack = BMSDPack::calc_check_sum_bmsd(&mut test_command[1]);
    assert_eq!(0xC3, test.crc);
    let test:&mut BMSDPack = BMSDPack::calc_check_sum_bmsd(&mut test_command[2]);
    assert_eq!(0x71, test.crc);
}


#[test]
fn test_calc_check_sum_awd(){
    let mut test_msg = AWDPack{
        address: 0x05,
        cmdcode: 0x87,
        param1: 0x0F,
        param0: 0x00,
        data1: 0x00,
        data0: 0x00,
        status: 0x00,
        cs: 0x00,
    };
    let test_cs = AWDPack::calc_check_sum_awd(&mut test_msg);
    println!("TEST AWD: {:X}", test_cs.cs);
    assert_eq!(0x65, test_msg.cs);
}

#[test]
fn test_send_command_bmsd(){
    let mut command_pack_bmsd: [BMSDPack; 5] =
	[
	    BMSDPack {
		header: 0xE6,
		controller_communications_data: 0x00,
		command_code: 0xA3,
		data: 0x30,
		crc: 0x00
	    },
	    BMSDPack {
		header: 0xE6,
		controller_communications_data: 0x00,
		command_code: 0xA5,
		data: 0x24,
		crc: 0x00
	    },
	    BMSDPack {
		header: 0xE6,
		controller_communications_data: 0x00,
		command_code: 0xA6,
		data: 0x24,
		crc: 0x00
	    },
	    BMSDPack {
		header: 0xE6,
		controller_communications_data: 0x00,
		command_code: 0xA7,
		data: 0x01,
		crc: 0x00
	    },
	    BMSDPack {
		header: 0xE6,
		controller_communications_data: 0x00,
		command_code: 0x51,
		data: 0x00,
		crc: 0x00
	    }
	];
    for j in 0..4{
	println!("Лифт {}: ", j);
	for i in 0..5{
	    BMSDPack::send_msg_bmsd(&mut command_pack_bmsd[i], j);
	    println!("{:X} {:X} {:X} {:X} {:X}", command_pack_bmsd[i].header, command_pack_bmsd[i].controller_communications_data, command_pack_bmsd[i].command_code, command_pack_bmsd[i].data, command_pack_bmsd[i].crc);
	}
    }
}

#[test]
fn give_position_canbus_test(){
	let mut buf = GEFRANPack::give_position_canbus("vcan0").expect("Введите порт CAN");
	println!("{} {}", buf.x_result, buf.y_result);
}
fn open_serial_port(serial_path: &str){
    let time_mill = Duration::new(0,0);
    let setting = serialport::SerialPortSettings{
        baud_rate: 9600,
        data_bits: serialport::DataBits::Eight,
        flow_control: serialport::FlowControl::None,
        parity: serialport::Parity::None,
        stop_bits: serialport::StopBits::One,
        timeout: time_mill
    };
    let path = Path::new(serial_path);
    TTYPort::open(path, &setting).unwrap();
}


fn main() {

    // open_serial_port("/dev/ttyUSB0");
    // test_calc_check_sum_awd();
    // BMSDPack::action_bmsd("up",1);

    // let mut modbus_platform = Modbus::new_rtu("/dev/ttyUSB0", 9600, 'N', 8, 1).expect("Отсутствует порт RS485");
    
	// open_oven(&mut modbus_platform, 7);
    // send_msg_oven(&mut modbus_platform, "open", &7);
	// close_oven(&mut modbus_platform);

}
