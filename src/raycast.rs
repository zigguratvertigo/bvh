//! This module defines the RaycastResult struct and the Intersectable trait.

use ray::Ray;

/// A struct which is returned by the `intersects_triangle` method.
pub enum RaycastResult {
    /// Represents a raycast hit
    Hit {
        /// Distance from the ray origin to the intersection point.
        distance: f32,
    },
    /// Represents a raycast miss, so the intersectable was not hit by the ray
    Miss,
}

impl RaycastResult {
    /// Constructs an `RaycastResult`. `distance` should be set to positive infinity,
    /// if the intersection does not occur.
    pub fn hit(distance: f32) -> RaycastResult {
        RaycastResult::Hit { distance: distance }
    }

    // /// Returns the hit point of a [`RaycastResult`] if it was a `Hit`.
    // /// Takes the [`Ray`] and calculates the point by movign from its origin
    // /// along its direction by the distance of the [`RaycastResult`].
    // ///
    // /// [`Ray`]: struct.Ray.html
    // /// [`RaycastResult`]: struct.RaycastResult.html
    // ///
    // pub fn hit_point(&self, ray: &Ray) -> Point3<f32> {
    //     if let RaycastResult::Hit { distance } = *self {
    //         return ray.origin + ray.direction() * distance;
    //     }
    //     panic!("Tracing to get the hit point of a missed raycast.");
    // }
}

/// A trait implemented by all things that can be intersected by a [`Ray`].
/// This applies to all structs that represent a two-or-higher-dimensional shape.
///
/// [`Ray`]: struct.Ray.html
///
pub trait Intersectable {
    /// Returns whether or not the [`Intersectable`] and the [`Ray`] intersect.
    /// Might use a faster algorithm than `intersection` at the cost of not returning a distance.
    ///
    /// [`Intersectable`]: trait.Intersectable.html
    /// [`Ray`]: struct.Ray.html
    ///
    fn does_intersect(&self, ray: &Ray) -> bool {
        match self.intersection(ray) {
            RaycastResult::Hit { .. } => true,
            RaycastResult::Miss => false,
        }
    }

    /// Returns the [`RaycastResult`] for the [`Intersectable`] and the [`Ray`],
    /// if both intersect.
    ///
    /// [`RaycastResult`]: struct.RaycastResult.html
    /// [`Intersectable`]: trait.Intersectable.html
    /// [`Ray`]: struct.Ray.html
    ///
    fn intersection(&self, &Ray) -> RaycastResult;
}
