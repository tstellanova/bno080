


#[macro_export]
#[cfg(debug_assertions)]
#[cfg(test)]
macro_rules! debug_println {
    ($s:expr, $($tt:tt)*) => ({ })
}

#[macro_export]
#[cfg(debug_assertions)]
#[cfg(not(test))]
macro_rules! debug_println {
    ($($arg:tt)*) => ({
        use cortex_m_semihosting::hprintln;
        hprintln!($($arg)*).unwrap();
    });
}

#[macro_export]
#[cfg(not(debug_assertions))]
macro_rules! debug_println {
    ($s:expr, $($tt:tt)*) => ({  })
}
