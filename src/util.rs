#[inline]
pub fn limit_range<T>(min: T, max: T, value: T) -> T
where T: PartialOrd {
    if value > max {
        max
    }
    else if value < min {
        min
    } else {
        value
    }
}
