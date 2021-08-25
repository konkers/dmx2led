pub struct Ws2812<'a> {
    data: &'a mut [u8],
    num_leds: usize,
}

impl<'a> Ws2812<'a> {
    pub fn new(num_leds: usize, data: &'a mut [u8]) -> Self {
        assert_eq!(data.len(), 140 * 2 + num_leds * 12);
        Self { data, num_leds }
    }

    pub const fn buffer_len(num_leds: usize) -> usize {
        140 * 2 + num_leds * 12
    }
}
