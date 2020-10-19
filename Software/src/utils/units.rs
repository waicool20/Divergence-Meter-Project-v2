#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliVolts(pub i32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliAmps(pub i32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliAmpHours(pub i32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct MilliWatts(pub i32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct Celcius(pub f32);

#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct Minutes(pub u32);

pub trait U32Extensions {
    fn min(self) -> Minutes;
}

pub trait I32Extensions {
    fn mv(self) -> MilliVolts;
    fn ma(self) -> MilliAmps;
    fn mah(self) -> MilliAmpHours;
    fn mw(self) -> MilliWatts;
}

pub trait F32Extensions {
    fn c(self) -> Celcius;
}

impl U32Extensions for u32 {
    fn min(self) -> Minutes {
        Minutes(self)
    }
}

impl I32Extensions for i32 {
    fn mv(self) -> MilliVolts {
        MilliVolts(self)
    }

    fn ma(self) -> MilliAmps {
        MilliAmps(self)
    }

    fn mah(self) -> MilliAmpHours {
        MilliAmpHours(self)
    }

    fn mw(self) -> MilliWatts {
        MilliWatts(self)
    }
}

impl F32Extensions for f32 {
    fn c(self) -> Celcius {
        Celcius(self)
    }
}

