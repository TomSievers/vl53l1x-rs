//! Module containing threshold definitions and implementations.

/// Threshold window behavior.
pub enum Window {
    /// Trigger when measurement is below threshold. The low threshold will be used as threshold.
    Below = 0,
    /// Trigger when measurement is above threshold. The high threshold will be used as threshold.
    Above = 1,
    /// Trigger when measurement is inside the window.
    In = 2,
    /// Trigger when measurement is outside the window.
    Out = 3,
}

impl From<u8> for Window {
    fn from(v: u8) -> Self {
        match v {
            0 => Window::Below,
            1 => Window::Above,
            2 => Window::In,
            3 => Window::Out,
            _ => Window::Below,
        }
    }
}

/// Threshold sturcture defining a threshold.
pub struct Threshold {
    /// The lower bound of the threshold window.
    pub low: u16,
    /// The upper bound of the threshold window.
    pub high: u16,
    /// The window thresholding behavior.
    pub window: Window,
}

impl Threshold {
    /// Create a new threshold with the given low threshold, high threshold and applied to the given window.
    ///
    /// # Arguments
    ///
    /// * `low` - Lower bound of the threshold window.
    /// * `high` - Upper bound of the threshold window.
    /// * `window` - The window thresholding behavior.
    pub fn new(low: u16, high: u16, window: Window) -> Self {
        Threshold { low, high, window }
    }
}
