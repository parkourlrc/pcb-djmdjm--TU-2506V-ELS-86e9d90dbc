//! Encoder/Servo control

#[derive(Clone, Copy, PartialEq)]
pub enum Direction {
    Forward,
    Backwards,
}

const JOG_SPEED_MICRONS_PER_SECOND: i64 = 500;
const JOG_TICK_HZ: i64 = 1000;

pub struct JogController {
    active: bool,
    direction: Direction,
    remaining_pulses: u32,
    conversion_remainder: i64,
    pending_conversion_remainder: i64,
    rate_remainder: i64,
    last_tick_ms: i64,
}

impl JogController {
    const PULSE_NUMERATOR: i64 = crate::MOTOR_PPR.abs() * crate::DRIVE_RATIO_LEADSCREW.abs();
    const PULSE_DENOMINATOR: i64 = crate::LEADSCREW_PITCH.abs() * crate::DRIVE_RATIO_MOTOR.abs();
    const RATE_NUMERATOR: i64 = JOG_SPEED_MICRONS_PER_SECOND * Self::PULSE_NUMERATOR;
    const RATE_DENOMINATOR: i64 = JOG_TICK_HZ * Self::PULSE_DENOMINATOR;

    pub fn new() -> Self {
        Self {
            active: false,
            direction: Direction::Forward,
            remaining_pulses: 0,
            conversion_remainder: 0,
            pending_conversion_remainder: 0,
            rate_remainder: 0,
            last_tick_ms: 0,
        }
    }

    pub fn start(&mut self, distance_microns: i32, now_ms: i64) -> bool {
        if self.active || distance_microns == 0 {
            return false;
        }

        let scaled = distance_microns as i64 * Self::PULSE_NUMERATOR + self.conversion_remainder;
        let rounding = Self::PULSE_DENOMINATOR / 2;
        let signed_pulses = if scaled >= 0 {
            (scaled + rounding) / Self::PULSE_DENOMINATOR
        } else {
            (scaled - rounding) / Self::PULSE_DENOMINATOR
        };
        if signed_pulses == 0 {
            return false;
        }

        self.direction = if signed_pulses > 0 {
            Direction::Forward
        } else {
            Direction::Backwards
        };
        self.remaining_pulses = signed_pulses.unsigned_abs() as u32;
        self.pending_conversion_remainder = scaled - signed_pulses * Self::PULSE_DENOMINATOR;
        self.rate_remainder = 0;
        self.last_tick_ms = now_ms;
        self.active = true;
        true
    }

    pub fn cancel(&mut self) {
        if self.active {
            self.active = false;
            self.remaining_pulses = 0;
            self.conversion_remainder = 0;
            self.pending_conversion_remainder = 0;
            self.rate_remainder = 0;
        }
    }

    pub fn pulses_due(&mut self, now_ms: i64) -> u32 {
        if !self.active || now_ms <= self.last_tick_ms {
            return 0;
        }

        // Process one quantum and discard missed time. Catching up after a
        // delayed loop would produce an unintended high-speed burst.
        self.last_tick_ms = now_ms;
        self.rate_remainder += Self::RATE_NUMERATOR;
        let pulses = (self.rate_remainder / Self::RATE_DENOMINATOR) as u32;
        self.rate_remainder %= Self::RATE_DENOMINATOR;
        let pulses = pulses.min(self.remaining_pulses);
        self.remaining_pulses -= pulses;

        if self.remaining_pulses == 0 {
            self.active = false;
            self.conversion_remainder = self.pending_conversion_remainder;
        }
        pulses
    }

    pub fn is_active(&self) -> bool {
        self.active
    }

    pub fn direction(&self) -> Direction {
        self.direction
    }

    pub fn remaining_microns(&self) -> i32 {
        if !self.active {
            return 0;
        }

        let distance = (self.remaining_pulses as i64 * Self::PULSE_DENOMINATOR
            + Self::PULSE_NUMERATOR / 2)
            / Self::PULSE_NUMERATOR;
        match self.direction {
            Direction::Forward => distance as i32,
            Direction::Backwards => -(distance as i32),
        }
    }
}

impl From<Direction> for bool {
    fn from(val: Direction) -> Self {
        match val {
            Direction::Forward => true,
            Direction::Backwards => false,
        }
    }
}

pub struct Control {
    // XXX direction?
    feed_rate_micron_per_rev: i32,

    feed_per_rev_factor: i64,
    fractional_pulses_remaining: i64,
    last_direction: Direction,
}

impl Control {
    pub fn new() -> Self {
        Control {
            feed_rate_micron_per_rev: 0,
            feed_per_rev_factor: 0,
            fractional_pulses_remaining: 0,
            last_direction: Direction::Forward,
        }
    }
    pub fn feed_per_rev(&mut self, encoder_pulses: i32, _elapsed_ms: u32) -> (Direction, u32) {
        let mut t: i64 = encoder_pulses as i64 * self.feed_per_rev_factor;
        t += self.fractional_pulses_remaining;
        // Retain remainder for next round.
        let mut pulses: i32 = (t >> 32) as i32;
        t -= (pulses as i64) << 32;
        self.fractional_pulses_remaining = t;
        let mut direction: Direction = self.last_direction;
        #[allow(clippy::comparison_chain)]
        if pulses > 0 {
            direction = Direction::Forward;
        } else if pulses < 0 {
            pulses = -pulses;
            direction = Direction::Backwards;
        }
        self.last_direction = direction;
        // XXX acceleration and deceleration control.
        // XXX hystereis for direction control.
        (direction, pulses as u32)
    }
    pub fn get_feed_rate_micron_per_rev(&self) -> i32 {
        self.feed_rate_micron_per_rev
    }
    pub fn get_last_direction(&self) -> Direction {
        self.last_direction
    }
    pub fn get_fractional_pulses_remaining(&self) -> i64 {
        self.fractional_pulses_remaining
    }
    pub fn set_feed_rate_micron_per_rev(&mut self, feed: i32) {
        // XXX bounds checking.
        self.feed_rate_micron_per_rev = feed;
        self.fractional_pulses_remaining = 0;
        // XXX consider fixed point split; is 32.32 ideal?
        // Precalculate multiplication factor.
        // Pulses to fractional turns (32.32 fixed point).
        let mut t: i64 = 1 << 32;
        t *= crate::ENCODER_RATIO_SPINDLE;
        t /= crate::ENCODER_PPR * crate::ENCODER_RATIO_ENCODER;
        // Encoder turns to fractional leadscrew turns (32.32).
        t *= self.feed_rate_micron_per_rev as i64;
        t *= crate::DRIVE_RATIO_LEADSCREW;
        t /= crate::DRIVE_RATIO_MOTOR * crate::LEADSCREW_PITCH;
        // Leadscrew turns to encoder pulses (32.32).
        t *= crate::MOTOR_PPR;
        self.feed_per_rev_factor = t
    }
    pub fn set_feed_rate_tpi(&mut self, tpi: i32) {
        // XXX bounds checking.
        self.feed_rate_micron_per_rev = 2540 / tpi; // Approximation only.
        self.fractional_pulses_remaining = 0;
        // XXX consider fixed point split; is 32.32 ideal?
        // Precalculate multiplication factor.
        // Pulses to fractional turns (32.32 fixed point).
        let mut t: i64 = 1 << 32;
        t *= crate::ENCODER_RATIO_SPINDLE;
        t /= crate::ENCODER_PPR * crate::ENCODER_RATIO_ENCODER;
        // Encoder turns to fractional leadscrew turns (32.32).
        t *= 25400;
        t /= tpi as i64;
        t *= crate::DRIVE_RATIO_LEADSCREW;
        t /= crate::DRIVE_RATIO_MOTOR * crate::LEADSCREW_PITCH;
        // Leadscrew turns to encoder pulses (32.32).
        t *= crate::MOTOR_PPR;
        self.feed_per_rev_factor = t
    }
}
