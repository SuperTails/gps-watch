use tinyvec::ArrayVec;

// TODO: enforce size based on key
#[derive(Copy, Clone)]
pub enum CfgItem {
    U1(u32, u8),
    U2(u32, u16),
    U4(u32, u32),
    U8(u32, u64),
}

impl CfgItem {
    pub fn packed_size(&self) -> usize {
        match self {
            CfgItem::U1(_, _) => 5,
            CfgItem::U2(_, _) => 6,
            CfgItem::U4(_, _) => 8,
            CfgItem::U8(_, _) => 12,
        }
    }

    pub fn to_bytes(&self) -> impl Iterator<Item = u8> {
        let mut arr = ArrayVec::<[u8; 12]>::new();
        match *self {
            CfgItem::U1(key, val) => {
                arr.extend(key.to_le_bytes());
                arr.extend(val.to_le_bytes());
            }
            CfgItem::U2(key, val) => {
                arr.extend(key.to_le_bytes());
                arr.extend(val.to_le_bytes());
            }
            CfgItem::U4(key, val) => {
                arr.extend(key.to_le_bytes());
                arr.extend(val.to_le_bytes());
            }
            CfgItem::U8(key, val) => {
                arr.extend(key.to_le_bytes());
                arr.extend(val.to_le_bytes());
            }
        }
        arr.into_iter()
    }
}

impl From<(u32, bool)> for CfgItem {
    fn from((key, val): (u32, bool)) -> Self {
        CfgItem::U1(key, val as u8)
    }
}

impl From<(u32, u8)> for CfgItem {
    fn from((key, val): (u32, u8)) -> Self {
        CfgItem::U1(key, val)
    }
}

impl From<(u32, u16)> for CfgItem {
    fn from((key, val): (u32, u16)) -> Self {
        CfgItem::U2(key, val)
    }
}

impl From<(u32, u32)> for CfgItem {
    fn from((key, val): (u32, u32)) -> Self {
        CfgItem::U4(key, val)
    }
}

impl From<(u32, u64)> for CfgItem {
    fn from((key, val): (u32, u64)) -> Self {
        CfgItem::U8(key, val)
    }
}

pub const CFG_UART1OUTPROT_UBX: u32 = 0x1074_0001;
pub const CFG_UART1OUTPROT_NMEA: u32 = 0x1074_0002;

pub const CFG_MSGOUT_UBX_NAV_PVT_UART1: u32 = 0x2091_0007;

pub const CFG_PM_OPERATEMODE: u32 = 0x20d0_0001;
pub const OPERATEMODE_FULL: u8 = 0;
pub const OPERATEMODE_PSMOO: u8 = 1;
pub const OPERATEMODE_PSMCT: u8 = 2;
pub const CFG_PM_POSUPDATEPERIOD: u32 = 0x40d0_0002;
pub const CFG_PM_ACQPERIOD: u32 = 0x40d0_0003;
pub const CFG_PM_ONTIME: u32 = 0x30d0_0005;
pub const CFG_PM_WAITTIMEFIX: u32 = 0x10d0_0009;
pub const CFG_PM_UPDATEEPH: u32 = 0x10d0_000a;
pub const CFG_PM_EXTINTBACKUP: u32 = 0x10d0_000d;
