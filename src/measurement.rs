use fixed::types::{I16F16, I0F8};

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    None = 0,
    SigmaFail = 1,
    SignalFail = 2,
    MinRangeClipped = 3,
    AboveMaxRange = 4,
    OutOfBounds = 5,
    HardwareFail = 6,
    WrapUnverified = 7,
    WrappedTarget = 8,
    ProcessingFail = 9,
    XTalkSignal = 10,
    Sync = 11,
    MergedPulses = 12,
    LackOfSignal = 13,
    MinRangeFailed = 14,
    RoiInvalid = 15,
    RangeInvalid = 255
}

#[derive(Clone, Copy)]
pub struct Measurement {
    stream_count : u8,
    signal_rate : I16F16,
    ambient_rate : I16F16,
    effective_spad_cnt : I0F8,
    sigma : u16,
    range : u16,
    status : Status,
}

impl Measurement {
    pub fn stream_count(&self) -> u8 {
        self.stream_count
    }

    pub fn signal_rate(&self) -> I16F16 {
        self.signal_rate
    }

    pub fn ambient_rate(&self) -> I16F16 {
        self.ambient_rate
    }

    pub fn effective_spad_cnt(&self) -> I0F8 {
        self.effective_spad_cnt
    }

    pub fn sigma(&self) -> u16 {
        self.sigma
    }

    pub fn range(&self) -> u16 {
        self.range
    }

    pub fn range_status(&self) -> Status {
        self.status
    }
}
