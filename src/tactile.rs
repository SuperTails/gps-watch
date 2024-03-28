use defmt::Format;
use stm32l4xx_hal::{gpio::ExtiPin, hal::digital::InputPin};

pub struct Joystick<L, R, U, D, C> {
    pub pin_left: L,
    pub pin_right: R,
    pub pin_up: U,
    pub pin_down: D,
    pub pin_center: C,
}

impl<L, R, U, D, C> Joystick<L, R, U, D, C>
where
    L: InputPin + ExtiPin,
    R: InputPin + ExtiPin,
    U: InputPin + ExtiPin,
    D: InputPin + ExtiPin,
    C: InputPin + ExtiPin,
{
    pub fn next_edge(&mut self) -> Option<Button> {
        if self.pin_left.check_interrupt() {
            self.pin_left.clear_interrupt_pending_bit();
            Some(Button::Left)
        } else if self.pin_right.check_interrupt() {
            self.pin_right.clear_interrupt_pending_bit();
            Some(Button::Right)
        } else if self.pin_up.check_interrupt() {
            self.pin_up.clear_interrupt_pending_bit();
            Some(Button::Up)
        } else if self.pin_down.check_interrupt() {
            self.pin_down.clear_interrupt_pending_bit();
            Some(Button::Down)
        } else if self.pin_center.check_interrupt() {
            self.pin_center.clear_interrupt_pending_bit();
            Some(Button::Center)
        } else {
            None
        }
    }
}

#[derive(Debug, Format, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    Left,
    Right,
    Up,
    Down,
    Center,
}
