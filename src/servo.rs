use rppal::i2c::Result;

pub struct Servo {
    // Quantities expressed as duty cycle
    center:f64,
    limit_radius: f64,
    dc_per_rad:f64, // Factor to translate from angle in radians to duty cycle
}

impl Servo {

    pub fn new(center:f64, limit_radius:f64, dc_per_rad:f64) -> Result<Servo> {
        Ok(Servo{center, limit_radius, dc_per_rad})
    }

    pub fn get_dt_from_angle(&self, angle:f64) -> Result<f64> {
        let mut output = self.center + angle * self.dc_per_rad;
        if output > self.center + self.limit_radius {
            output = self.center + self.limit_radius;
        }
        if output < self.center + self.limit_radius {
            output = self.center - self.limit_radius;
        }
        Ok(output)
    }

}
