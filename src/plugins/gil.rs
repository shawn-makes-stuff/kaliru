// Process-wide state of the plugin layer. Python plugins call kaliru from
// their own threads too (register_async_callback), so the layer's tables
// cannot be thread-local. Every access happens with the GIL held, which
// serializes the reactor thread and python threads (docs/PLUGINS.md "GIL
// and threads"): that is what makes these statics sound.
use std::sync::OnceLock;

pub struct Gil<T> {
    cell: OnceLock<T>,
    init: fn() -> T,
}

// SAFETY: only used with the GIL held (see above)
unsafe impl<T> Sync for Gil<T> {}

impl<T> Gil<T> {
    pub const fn new(init: fn() -> T) -> Self {
        Gil { cell: OnceLock::new(), init }
    }

    pub fn with<R>(&self, f: impl FnOnce(&T) -> R) -> R {
        f(self.cell.get_or_init(self.init))
    }
}
