use embedded_hal::digital::v2::{toggleable, OutputPin, StatefulOutputPin};

/// An output pin that does not have any physical basis.
/// This can be used, for example, when you know that there is no physical trace
/// connecting an MCU to a sensor, and you want the driver to proceed as if there were.
///
pub struct DummyOutputPin {
    state: bool,
}

impl DummyOutputPin {
    pub fn new() -> Self {
        Self { state: false }
    }
}

impl OutputPin for DummyOutputPin {
    type Error = core::convert::Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.state = false;
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.state = true;
        Ok(())
    }
}

impl StatefulOutputPin for DummyOutputPin {
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(!self.state)
    }
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        Ok(self.state)
    }
}

/// Opt-in to the software implementation.
impl toggleable::Default for DummyOutputPin {}
