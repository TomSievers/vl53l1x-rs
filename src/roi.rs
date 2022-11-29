//! Module containing region of interest definitions.

/// Structure denoting a region of interest.
pub struct ROI {
    /// Width of the region of interest.
    pub width: u16,
    /// Height of the region of interest.
    pub height: u16,
}

impl ROI {
    /// Create a new region of interest with the given width and height.
    ///
    /// # Arguments
    ///
    /// * `width` - The width of the ROI.
    /// * `height` - The height of the ROI.
    pub fn new(width: u16, height: u16) -> ROI {
        ROI { width, height }
    }
}

/// Structure denoting the center of a region of interest.
pub struct ROICenter {
    /// The SPAD used as center point.
    pub spad: u8,
}

impl ROICenter {
    /// Create a new center point from the given coordinate.
    ///
    /// # Arguments
    ///
    /// * `x` - Horizontal position of the center coordinate.
    /// * `y` - Vertical position of the center coordinate.
    pub fn new(x: u8, y: u8) -> Self {
        let spad = if y > 7 {
            128 + (x << 3) + (15 - y)
        } else {
            ((15 - x) << 3) + y
        };

        ROICenter { spad }
    }
}
