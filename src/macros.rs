

/// Debug print macro that is defined to an empty macro unless feature `dbgoprint` is enabled
///
#[cfg(any(not(debug_assertions), test))]
#[macro_export]
macro_rules! debug_println {
    ($($arg:tt)*) =>  {{}};
}

/// Debug print macro that is defined to an empty macro unless feature `dbgoprint` is enabled
///
#[cfg(all(debug_assertions, not(test)))]
#[macro_export]
macro_rules! debug_println {
    ($($arg:tt)*) => ({
        use cortex_m_semihosting::hprintln;
        hprintln!($($arg)*).unwrap();
    });
}




