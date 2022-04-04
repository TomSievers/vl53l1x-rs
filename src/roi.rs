pub struct Point {
    x : u8,
    y : u8,
}

impl Point {
    pub fn new(x : u8, y : u8) -> Point {
        Point { 
            x, 
            y 
        }
    }

    pub fn x(&self) -> u8 {
        self.x
    }

    pub fn y(&self) -> u8 {
        self.y
    }
}

pub struct Roi {
    top_left : Point,
    bottom_right : Point,
}

impl Roi {
    pub fn new(top_left : Point, bottom_right : Point) -> Self {
        Roi {
            top_left,
            bottom_right,
        }
    }

    pub fn new_simple(width : u8, height : u8) -> Self {
        Roi {
            top_left : Point::new(0, 0),
            bottom_right : Point::new(width, height),
        }
    }
}