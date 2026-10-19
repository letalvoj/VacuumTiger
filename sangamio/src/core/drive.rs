//! Drive rules every driver applies, the simulated one included.

/// The bumper interlock: driving forward into a pressed (front) bumper stops the wheels at
/// once, turn and all. Backing off and turning on the spot pass: that is how a controller
/// gets free again (the corner sweep touches walls on purpose), as in Kobuki's safety
/// controller. `linear` is in any unit; the caller makes forward positive.
pub fn bumper_stops(bumper_pressed: bool, linear: f32) -> bool {
    bumper_pressed && linear > 0.0
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_forward_into_a_pressed_bumper_stops() {
        assert!(bumper_stops(true, 0.1));
        assert!(!bumper_stops(true, 0.0), "turning on the spot");
        assert!(!bumper_stops(true, -0.1), "backing off");
        assert!(!bumper_stops(false, 0.1));
    }
}
