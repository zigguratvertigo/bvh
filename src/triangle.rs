//! This module defines a Triangle structure.
//! Triangles implement Bounded and Intersectable.

use EPSILON;
use aabb::{AABB, Bounded};
use nalgebra::{Point3, Cross, Dot};
use ray::Ray;
use raycast::{Intersectable, RaycastResult};

/// A triangle struct. Instance of a more complex `Bounded` primitive.
pub struct Triangle {
    a: Point3<f32>,
    b: Point3<f32>,
    c: Point3<f32>,
    aabb: AABB,
}

impl Triangle {
    /// Creates a new [`Triangle`] with the three given points.
    ///
    /// [`Triangle`]: struct.Triangle.html
    ///
    pub fn new(a: Point3<f32>, b: Point3<f32>, c: Point3<f32>) -> Triangle {
        Triangle {
            a: a,
            b: b,
            c: c,
            aabb: AABB::empty().grow(&a).grow(&b).grow(&c),
        }
    }

    /// Returns the first vertex of the [`Triangle`].
    ///
    /// [`Triangle`]: struct.Triangle.html
    ///
    pub fn a(&self) -> Point3<f32> {
        self.a
    }

    /// Returns the second vertex of the [`Triangle`].
    ///
    /// [`Triangle`]: struct.Triangle.html
    ///
    pub fn b(&self) -> Point3<f32> {
        self.b
    }

    /// Returns the third vertex of the [`Triangle`].
    ///
    /// [`Triangle`]: struct.Triangle.html
    ///
    pub fn c(&self) -> Point3<f32> {
        self.c
    }
}

impl Bounded for Triangle {
    fn aabb(&self) -> AABB {
        self.aabb
    }
}

impl Intersectable for Triangle {
    /// Implementation of the [Möller-Trumbore triangle/ray intersection algorithm]
    /// (https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm).
    fn intersection(&self, ray: &Ray) -> RaycastResult {
        let a = self.a;
        let b = self.b;
        let c = self.c;

        let a_to_b = b - a;
        let a_to_c = c - a;

        // Begin calculating determinant - also used to calculate u parameter
        // u_vec lies in view plane
        // length of a_to_c in view_plane = |u_vec| = |a_to_c|*sin(a_to_c, dir)
        let u_vec = ray.direction().cross(&a_to_c);

        // If determinant is near zero, ray lies in plane of triangle
        // The determinant corresponds to the parallelepiped volume:
        // det = 0 => [dir, a_to_b, a_to_c] not linearly independant
        let det = a_to_b.dot(&u_vec);

        // Only testing positive bound, thus enabling backface culling
        // If backface culling is not desired write:
        // det < EPSILON && det > -EPSILON
        if det < EPSILON {
            return RaycastResult::Miss;
        }

        let inv_det = 1.0 / det;

        // Vector from point a to ray origin
        let a_to_origin = ray.origin - a;

        // Calculate u parameter
        let u = a_to_origin.dot(&u_vec) * inv_det;

        // Test bounds: u < 0 || u > 1 => outside of triangle
        if u < 0.0 || u > 1.0 {
            return RaycastResult::Miss;
        }

        // Prepare to test v parameter
        let v_vec = a_to_origin.cross(&a_to_b);

        // Calculate v parameter and test bound
        let v = ray.direction().dot(&v_vec) * inv_det;
        // The intersection lies outside of the triangle
        if v < 0.0 || u + v > 1.0 {
            return RaycastResult::Miss;
        }

        let dist = a_to_c.dot(&v_vec) * inv_det;

        if dist > EPSILON {
            // RaycastResult::new(dist, u, v)
            RaycastResult::hit(dist)
        } else {
            RaycastResult::Miss
        }
    }
}
