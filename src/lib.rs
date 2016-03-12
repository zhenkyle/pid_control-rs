//! Software PID controller
//!
//! Owes a great debt to:
//!
//! * http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
//! * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

extern crate core;

use std::f64;

pub struct PIDController {
    /// Proportional gain
    pub p_gain: f64,

    /// Integral gain
    pub i_gain: f64,

    /// Differential gain,
    pub d_gain: f64,

    /// Collected i
    pub i_state: f64,

    /// Previous input value
    pub prev_value: f64,

    /// Output range limits
    pub i_min: f64,
    pub i_max: f64,
}

#[inline]
fn limit_range<T>(min: T, max: T, value: T) -> T
where T: PartialOrd {
    if value > max {
        max
    }
    else if value < min {
        min
    } else {
        value
    }
}

impl PIDController {
    pub fn new(p_gain: f64, i_gain: f64, d_gain: f64) -> PIDController {
        PIDController{
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,
            i_state: 0.0,
            prev_value: 0.0,
            i_min: -f64::INFINITY,
            i_max: f64::INFINITY,
        }
    }

    pub fn update(&mut self, error: f64, value: f64) -> f64 {
        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.i_state += error;
        let i_term = limit_range(self.i_min, self.i_max,
                                 self.i_gain * self.i_state);

        // DIFFERENTIAL
        let d_term = self.d_gain * (self.prev_value - value);
        self.prev_value = value;

        p_term + d_term + i_term
    }
}
