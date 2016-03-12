//! Software PID controller
//!
//! Owes a great debt to:
//!
//! * https://en.wikipedia.org/wiki/PID_controller
//! * http://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
//! * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/

extern crate core;

use std::f64;

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

#[derive(Debug, Clone)]
pub struct PIDController {
    /// Proportional gain
    pub p_gain: f64,

    /// Integral gain
    pub i_gain: f64,

    /// Differential gain,
    pub d_gain: f64,

    pub target: f64,

    /// Collected i
    pub err_sum: f64,

    /// Previous input value
    pub prev_error: f64,

    /// Output range limits
    pub i_min: f64,
    pub i_max: f64,
}

impl PIDController {
    pub fn new(p_gain: f64, i_gain: f64, d_gain: f64) -> PIDController {
        PIDController{
            p_gain: p_gain,
            i_gain: i_gain,
            d_gain: d_gain,

            target: 0.0,

            err_sum: 0.0,
            prev_error: 0.0,

            i_min: -f64::INFINITY,
            i_max: f64::INFINITY,
        }
    }

    pub fn update(&mut self, value: f64, delta_t: f64) -> f64 {
        let error = self.target - value;

        // PROPORTIONAL
        let p_term = self.p_gain * error;

        // INTEGRAL
        self.err_sum += limit_range(
            self.i_min, self.i_max,
            self.err_sum + error * delta_t
        );
        let i_term = self.i_gain * self.err_sum;

        // DIFFERENTIAL
        let d_term = self.d_gain * (self.prev_error - error) / delta_t;
        self.prev_error = error;

        p_term + d_term + i_term
    }
}
