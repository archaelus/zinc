
use super::regs::reg::*;

#[inline(always)]
pub fn init() {
    if RTC.cr.osce() == Rtc_cr_osce::Disabled { // If RTC OSC is disabled
        RTC.sr
            .set_tce(Rtc_sr_tce::Disabled); // Disable Time Counter
        // Add 20pf load to RTC clock, and enable the RTC oscillator
        RTC.cr.ignoring_state()
            .set_sc16p(true)
            .set_sc4p(true)
            .set_osce(Rtc_cr_osce::Enabled);
    }
}

/// Control the RTC Seconds Interrupt (true enabled the interrupt).
pub fn tick_isr(enabled: bool) {
    match enabled {
        true => {
            RTC.ier.set_tsie(Rtc_ier_tsie::Enabled);
        }
        false => {
            RTC.ier.set_tsie(Rtc_ier_tsie::Disabled);
        }
    };
}
