// TODO: actually implement a triple buffer.
// For now we use some unsafe hyjinx to bypass the borrow checker.
pub struct TripleBuffer<T: Default + Copy + 'static, const N: usize> {
    buf: &'static mut [T; N], /*
                              bufs: [[T; N]; 3],
                              write_index: AtomicUsize,
                              stage_index: AtomicUsize,
                              read_index: AtomicUsize,
                              stage_full: AtomicBool,
                              */
}

pub struct TripleBufferReader<T: Default + Copy, const N: usize> {
    buf: *mut [T; N],
    /*
    parent: *mut [T; N],
    */
}

impl<T: Copy + Default, const N: usize> TripleBuffer<T, N> {
    pub fn new(buf: &'static mut [T; N]) -> (Self, TripleBufferReader<T, N>) {
        let buf = TripleBuffer {
            buf, /*
                     bufs: [[T::default(); N], [T::default(); N], [T::default(); N]],
                     write_index: AtomicUsize::new(0),
                     stage_index: AtomicUsize::new(1),
                     read_index: AtomicUsize::new(2),
                     stage_full: AtomicBool::new(false),
                 */
        };

        let reader = TripleBufferReader {
            buf: buf.buf as *mut [T; N],
            /*
                    parent: &mut buf as *mut TripleBuffer<T, N>,
            */
        };

        (buf, reader)
    }

    pub fn set(&mut self, index: usize, val: T) {
        if index < N {
            self.buf[index] = val;
        }
    }

    pub fn sync(&mut self) {
        /*
            let write_index = self.stage_index.load(Ordering::Relaxed);
            loop {
                let stage_index = self.stage_index.load(Ordering::Relaxed);

            }
        */
    }
}

impl<T: Copy + Default, const N: usize> TripleBufferReader<T, N> {
    pub fn get(&mut self, index: usize) -> T {
        if index < N {
            unsafe { (*self.buf)[index] }
        } else {
            T::default()
        }
    }

    pub fn sync(&mut self) {
        /*
            let write_index = self.stage_index.load(Ordering::Relaxed);
            loop {
                let stage_index = self.stage_index.load(Ordering::Relaxed);

            }
        */
    }
}

unsafe impl<T: Copy + Default, const N: usize> Send for TripleBufferReader<T, N> {}
