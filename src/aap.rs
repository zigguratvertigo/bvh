//! Axis Aligned Planes.

use axis::Axis;
use nalgebra::Point3;
use std::f32;

/// AAP struct.
#[derive(Debug, Copy, Clone)]
pub struct AAP {
    /// The axis that is perpendicular to this plane
    axis: usize,

    /// The offset of the plane relative to 0,0,0, along the axis
    pub offset: f32,
}

impl AAP {
    /// Creates a new [`AAP`] with the given parameters.
    ///
    /// # Examples
    /// ```
    /// use bvh::aap::AAP;
    /// use bvh::aabb::Axis;
    ///
    /// let aap = AAP::new(Axis::X, 10.0);
    /// // TODO Finish example
    /// ```
    ///
    /// [`AAP`]: struct.AAP.html
    ///
    pub fn new(axis: Axis, offset: f32) -> AAP {
        AAP {
            axis: axis as usize,
            offset: offset,
        }
    }

    /// Gets the axis of the [`AAP`].
    ///
    /// [`AAP`]: struct.AAP.html
    ///
    pub fn axis(&self) -> usize {
        self.axis
    }

    // TODO Think of better name
    /// Returns `true` if the [`Point3`] is "right" of the [`AAP`].
    /// This is the case when the [`Point3`]'s component on the [`AAP`]'s axis is equal or higher
    /// than the [`AAP`]'s `offset`.
    ///
    /// TODO example
    ///
    /// [`AAP`]: struct.AAP.html
    /// [`Point3`]: http://nalgebra.org/doc/nalgebra/struct.Point3.html
    ///
    pub fn has_point_to_right(&self, p: &Point3<f32>) -> bool {
        p[self.axis] >= self.offset
    }

    // TODO Think of better name
    /// Returns `true` if the [`Point3`] is "left" of the [`AAP`].
    /// This is the case when the [`Point3`]'s component on the [`AAP`]'s axis is equal or lower
    /// than the [`AAP`]'s `offset`.
    ///
    /// TODO example
    ///
    /// [`AAP`]: struct.AAP.html
    /// [`Point3`]: http://nalgebra.org/doc/nalgebra/struct.Point3.html
    ///
    pub fn has_point_to_left(&self, p: &Point3<f32>) -> bool {
        p[self.axis] <= self.offset
    }

    // TODO Think of better name
    /// Returns `true` if the [`Point3`] is on the [`AAP`].
    /// This is the case when the [`Point3`]'s component on the [`AAP`]'s axis is equal or higher
    /// than the [`AAP`]'s `offset`.
    ///
    /// TODO example
    ///
    /// [`AAP`]: struct.AAP.html
    /// [`Point3`]: http://nalgebra.org/doc/nalgebra/struct.Point3.html
    ///
    pub fn has_point_on_it(&self, p: &Point3<f32>) -> bool {
        p[self.axis] == self.offset
    }
}

/// Default instance for [`AAP`]s. Returns an [`AAP`] that is perpendicular to the X axis
/// and goes through 0,0,0.
///
/// [`AAP`]: struct.AAP.html
///
impl Default for AAP {
    fn default() -> AAP {
        AAP::new(Axis::X, 0.0)
    }
}

// TODO Write tests
