use rppal::i2c::Result;

use std::thread::sleep;
use std::time::Duration;


pub mod servo;


use rppal::i2c;


const RESOLUTION:u16 = 4096;
const MODE1_ADDR:u8 = 0;
const MODE1_SLEEP:u8 = 0x10;
const MODE1_RESTART:u8 = 0x80;
const MODE1_AI:u8 = 0x20;
const ALL_OFF_H_ADDR:u8 = 0xfd;
const ALL_OFF_H_ALL_OFF:u8 = 0x10;   // Set all-off bit to 1
const PRESCALE_ADDR:u8 = 0xfe;
const OSCILLATOR_FREQ:f64 = 25e6;   // As recommended by LadyAda


struct TwoBytes {
    pub low:u8,
    pub high:u8,
}


pub struct ServoDriver {
    i2c_port : i2c::I2c,
}


fn get_two_bytes(word:u16) -> TwoBytes {
    let low = (word & 0xFF) as u8;
    let high = ((word & 0xFF00) >> 8) as u8;
    TwoBytes{low,high}
}


impl ServoDriver {
    pub fn new(addr:u16, freq: f64) -> Result<ServoDriver> {
        let mut i2c = i2c::I2c::new()?;
        i2c.set_slave_address(addr)?;
        let mut servo = ServoDriver{i2c_port : i2c};
        servo.initialize(freq)?;
        Ok(servo)
    }


    fn reset(&mut self) -> Result<()> {
        let message:Vec<u8> = vec![MODE1_ADDR, MODE1_RESTART];
        self.i2c_port.write(&message[0..2])?;

        Ok(())
    }


    fn initialize(&mut self, freq:f64) -> Result<()> {
        self.reset()?;
        self.set_prescale(freq)?;
        self.set_mode()?;

        Ok(())
    }


    fn set_prescale(&mut self, freq:f64) -> Result<()> {

        // Compute prescale factor
        let prescale_factor = ((OSCILLATOR_FREQ / ((RESOLUTION as f64) * freq)) - 1.0).round() as u8;

        let mut buffer:Vec<u8> = vec![MODE1_ADDR, 0];

        // Read old mode 1
        self.read_reg(MODE1_ADDR, &mut buffer[1..2])?;
        let old_mode = buffer[1];

        // Write sleeping state
        let new_mode = (old_mode & !MODE1_RESTART) | MODE1_SLEEP;
        buffer[1] = new_mode;
        self.i2c_port.write(&buffer[0..2])?;

        // Write prescale
        let message : Vec<_> = vec![PRESCALE_ADDR, prescale_factor];
        self.i2c_port.write(&message[0..2])?;

        // Restore previous mode
        buffer[1] = old_mode;
        self.i2c_port.write(&buffer[0..2])?;

        // Wait 5 ms
        sleep(Duration::from_millis(5));

        Ok(())
    }


    fn set_mode(&mut self) -> Result<()> {
        let message:Vec<u8> = vec![MODE1_ADDR, MODE1_RESTART|MODE1_AI];
        self.i2c_port.write(&message[0..2])?;

        Ok(())
    }


    pub fn set_duty_cycle(&mut self, motor:u8, duty_cycle:f64) -> Result<()> {
        let mut dc = duty_cycle;
        if duty_cycle > 1.0 {
            dc = 1.0;
        }
        else if duty_cycle < 0.0 {
            dc = 0.0;
        }

        // Get first register for the selected output
        let register = (6 + 4 * motor) as u8;

        // To set the frequency, start at zero and stop at a fitting step
        let stop_step : u16 = (RESOLUTION as f64 * dc) as u16;
        let stop_step_bytes = get_two_bytes(stop_step);

        let message = vec![register, 0x0, 0x0, stop_step_bytes.low, stop_step_bytes.high];
        self.i2c_port.write(&message[0..5])?;

        Ok(())
    }


    pub fn all_off(&mut self) -> Result<()> {
        let message : Vec<_> = vec![ALL_OFF_H_ADDR, ALL_OFF_H_ALL_OFF];
        self.i2c_port.write(&message[0..2])?;
        sleep(Duration::from_millis(5));

        Ok(())
    }


    pub fn read_reg(&mut self, addr:u8, buffer:&mut [u8]) -> Result<()> {
        buffer[0] = addr;
        let buffer_len = buffer.len();
        self.i2c_port.write(&buffer[0..1])?;
        self.i2c_port.read(&mut buffer[0..buffer_len])?;

        Ok(())
    }


    pub fn pwm_sleep(&mut self) -> Result<()> {
        let mut buffer:Vec<u8> = vec![MODE1_ADDR, 0];
        self.read_reg(MODE1_ADDR, &mut buffer[1..2])?;
        let asleep: u8 = buffer[1] | MODE1_SLEEP;
        buffer[1] = asleep;
        self.i2c_port.write(&buffer[0..2])?;
        sleep(Duration::from_millis(5));

        Ok(())
    }
}
