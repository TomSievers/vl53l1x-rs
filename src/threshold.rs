use fixed::types::I16F16;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    None,
    Distance,
    Rate,
    DistanceAndRate,
    DistanceOrRate
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Type {
    CrossedLow,
    CrossedHigh,
    InsideWindow,
    OutsideWindow
}

pub struct Threshold {
    t : Type,
    low_mm : Option<u16>,
    high_mm : Option<u16>,
    low_rate : Option<I16F16>,
    high_rate : Option<I16F16>,
}

impl Threshold {
    pub fn distance_threshold(high : u16, low : u16, t : Type) -> Self {
        Threshold {
            t,
            high_mm : Some(high),
            low_mm : Some(low),
            low_rate : None,
            high_rate : None,
        }
    }

    pub fn rate_threshold(high : I16F16, low : I16F16, t : Type) -> Self {
        Threshold {
            t,
            high_mm : None,
            low_mm : None,
            low_rate : Some(low),
            high_rate : Some(high),
        }
    }
}

pub enum Trigger {
    TargetFound,
    NoTarget
}

pub struct ThresholdSettings {
    mode : Mode,
    dist_threshold : Option<Threshold>,
    rate_threshold : Option<Threshold>,
    trigger : Trigger,
}

impl ThresholdSettings {
    pub fn default() -> Self {
        ThresholdSettings {
            mode : Mode::None,
            dist_threshold : None,
            rate_threshold : None,
            trigger : Trigger::NoTarget
        }
    }

    pub fn mode(mut self, mode : Mode) -> Self {

        self.mode = mode;

        self
    }

    pub fn threshold(mut self, threshold : Threshold) -> Self {

        if threshold.high_mm.is_some() {
            self.dist_threshold = Some(threshold);
        } else {
            self.rate_threshold = Some(threshold);
        }

        self
    }

    pub fn trigger_on(mut self, trigger : Trigger) -> Self {

        self.trigger = trigger;

        self
    }
}