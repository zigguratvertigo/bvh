//! Axis enum for AABBs and AAPs.

use std::ops::Index;
use std::fmt::{Display, Formatter, Result};

/// An Axis in a three-dimensional coordinate system.
/// Used to access `Vector3`/`Point3` structs via index.
#[derive(Debug, Copy, Clone, PartialEq)]
pub enum Axis {
    /// Index of the X axis.
    X = 0,

    /// Index of the Y axis.
    Y = 1,

    /// Index of the Z axis.
    Z = 2,
}

/// Make slices indexable by Axes.
impl Index<Axis> for [f32] {
    type Output = f32;

    fn index(&self, axis: Axis) -> &f32 {
        self.index(axis as usize)
    }
}

/// TODO comment
///
impl Display for Axis {
    fn fmt(&self, f: &mut Formatter) -> Result {
        let c = match *self {
            Axis::X => "x",
            Axis::Y => "y",
            Axis::Z => "z",
        };
        write!(f, "{}", c)
    }
}
