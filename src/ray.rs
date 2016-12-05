//! This module defines a Ray structure.

use aabb::AABB;
use aap::AAP;
use nalgebra::{Vector3, Point3, Norm};
use std::f32::INFINITY;

/// A struct which defines a ray and some of its cached values.
#[derive(Debug)]
pub struct Ray {
    /// The ray origin.
    pub origin: Point3<f32>,

    /// The ray direction.
    direction: Vector3<f32>,

    /// Inverse (1/x) ray direction. Cached for use in [`AABB`] intersections.
    ///
    /// [`AABB`]: struct.AABB.html
    ///
    inv_direction: Vector3<f32>,

    /// Sign of the direction. 0 means positive, 1 means negative.
    /// Cached for use in [`AABB`] intersections.
    ///
    /// [`AABB`]: struct.AABB.html
    ///
    sign: Vector3<usize>,
}

impl Ray {
    /// Creates a new [`Ray`] from an `origin` and a `direction`.
    /// `direction` will be normalized.
    ///
    /// # Examples
    /// ```
    /// use bvh::ray::Ray;
    /// use bvh::nalgebra::{Point3,Vector3};
    ///
    /// let origin = Point3::new(0.0,0.0,0.0);
    /// let direction = Vector3::new(1.0,0.0,0.0);
    /// let ray = Ray::new(origin, direction);
    ///
    /// assert_eq!(ray.origin, origin);
    /// assert_eq!(ray.direction, direction);
    /// ```
    ///
    /// [`Ray`]: struct.Ray.html
    ///
    pub fn new(origin: Point3<f32>, direction: Vector3<f32>) -> Ray {
        let direction = direction.normalize();
        Ray {
            origin: origin,
            direction: direction,
            inv_direction: Vector3::new(1.0 / direction.x, 1.0 / direction.y, 1.0 / direction.z),
            sign: Vector3::new((direction.x < 0.0) as usize,
                               (direction.y < 0.0) as usize,
                               (direction.z < 0.0) as usize),
        }
    }

    /// Returns the [`Ray`]'s direction.
    ///
    pub fn direction(&self) -> Vector3<f32> {
        self.direction
    }

    /// Returns the [`Ray`]'s sign vector.
    /// For each axis, 0 means positive, 1 means negative.
    ///
    pub fn sign(&self) -> Vector3<usize> {
        self.sign
    }

    /// Returns the [`Ray`]'s inverse direction (1/direction).
    /// This value can be used for efficient division through the direction's components.
    ///
    pub fn inv_direction(&self) -> Vector3<f32> {
        self.inv_direction
    }

    /// Returns how long the [`Ray`] travels before it leaves the given [`AABB`].
    /// Reasonable results require ray.intersects_aabb(aabb) to return `true`.
    ///
    /// # Examples
    /// ```
    /// use bvh::aabb::AABB;
    /// use bvh::ray::Ray;
    /// use bvh::nalgebra::{Point3,Vector3};
    ///
    /// let origin = Point3::new(0.0,0.0,0.0);
    /// let direction = Vector3::new(1.0,0.0,0.0);
    /// let ray = Ray::new(origin, direction);
    ///
    /// let point1 = Point3::new(-1.0,-1.0,-1.0);
    /// let point2 = Point3::new(1.0,1.0,1.0);
    /// let aabb = AABB::with_bounds(point1, point2);
    ///
    /// let epsilon = 0.000001f32;
    ///
    /// let distance = ray.distance_to_aabb_end(&aabb);
    /// assert!((distance - 1.0).abs() < epsilon);
    /// ```
    ///
    /// [`Ray`]: struct.Ray.html
    /// [`AABB`]: struct.AABB.html
    ///
    pub fn distance_to_aabb_end(&self, aabb: &AABB) -> f32 {
        let mut min_distance = INFINITY;
        for axis in 0..3 {
            let origin = self.origin[axis];
            let far_side = aabb[1 - self.sign[axis]][axis];
            let distance = (origin - far_side).abs();
            let ray_distance = distance * self.inv_direction[axis];

            if min_distance < ray_distance {
                min_distance = ray_distance
            }
        }
        min_distance
    }

    /// Tests the intersection of a [`Ray`] with an [`AAP`].
    ///
    /// # Examples
    /// ```
    /// use bvh::ray::Ray;
    /// use bvh::nalgebra::{Point3,Vector3};
    ///
    /// let origin = Point3::new(0.0,0.0,0.0);
    /// let direction = Vector3::new(1.0,0.0,0.0);
    /// let ray = Ray::new(origin, direction);
    ///
    /// // TODO Finish example
    /// ```
    ///
    /// [`Ray`]: struct.Ray.html
    /// [`AAP`]: struct.AAP.html
    ///
    pub fn intersects_aap(&self, aap: AAP) -> bool {
        // True if the ray goes into the positive direction on the AAP's axis.
        let goes_right = self.sign[aap.axis()] == 0;
        // Return true if the origin lies on the opposite side of the AAP than the ray's direction.
        if goes_right {
            aap.has_point_to_left(&self.origin)
        } else {
            aap.has_point_to_right(&self.origin)
        }
    }

    // TODO currently unused
    /// Tests if part of the [`Ray`] is left of the [`AAP`].
    /// Either the [`Ray`] starts left of the [`AAP`], or it intersects it.
    ///
    /// # Examples
    /// ```
    /// use bvh::ray::Ray;
    /// use bvh::nalgebra::{Point3,Vector3};
    ///
    /// let origin = Point3::new(0.0,0.0,0.0);
    /// let direction = Vector3::new(1.0,0.0,0.0);
    /// let ray = Ray::new(origin, direction);
    ///
    /// // TODO Finish example
    /// ```
    ///
    /// [`Ray`]: struct.Ray.html
    /// [`AABB`]: struct.AABB.html
    ///
    pub fn is_left_of_aap(&self, aap: AAP) -> bool {
        aap.has_point_to_left(&self.origin) || self.direction[aap.axis()] < 0.0
    }

    // TODO currently unused
    /// Tests if part of the [`Ray`] is right of the [`AAP`].
    /// Either the [`Ray`] starts right of the [`AAP`], or it intersects it.
    ///
    /// # Examples
    /// ```
    /// use bvh::ray::Ray;
    /// use bvh::nalgebra::{Point3,Vector3};
    ///
    /// let origin = Point3::new(0.0,0.0,0.0);
    /// let direction = Vector3::new(1.0,0.0,0.0);
    /// let ray = Ray::new(origin, direction);
    ///
    /// // TODO Finish example
    /// ```
    ///
    /// [`Ray`]: struct.Ray.html
    /// [`AABB`]: struct.AABB.html
    ///
    pub fn is_right_of_aap(&self, aap: AAP) -> bool {
        aap.has_point_to_right(&self.origin) || self.direction[aap.axis()] > 0.0
    }

    // // TODO think of better name
    // /// Creates a new [`Ray`] that is on the same line as the given [`Ray`],
    // /// but has its origin moved onto the given [`AAP`].
    // ///
    // pub fn new_from_aap(ray: &Ray, aap: AAP) -> Ray {
    //     // Distance between ray origin and plane
    //     let origin_distance = aap.offset - ray.origin[aap.axis()];
    //     // How far along the ray's direction we have to move the origin
    //     let origin_delta = origin_distance / ray.direction[aap.axis()];
    //     Ray::new(ray.origin + (origin_delta * ray.direction), ray.direction)
    // }

    /// Creates a new [`Ray`] that is on the same line as the given [`Ray`],
    /// but has its origin moved along the direction by `offset`.
    ///
    /// [`Ray`]: struct.Ray.html
    ///
    pub fn new_with_moved_origin(ray: &Ray, offset: f32) -> Ray {
        Ray {
            origin: ray.point_along_direction(offset),
            direction: ray.direction,
            inv_direction: ray.inv_direction,
            sign: ray.sign,
        }
    }

    /// Calculates the point that is `offset` away from the [`Ray`]'s origin,
    /// when moving along its direction.
    ///
    /// [`Ray`]: struct.Ray.html
    ///
    pub fn point_along_direction(&self, offset: f32) -> Point3<f32> {
        self.origin + (offset * self.direction)
    }
}

#[cfg(test)]
mod tests {
    use EPSILON;
    use ray::Ray;
    use raycast::{Intersectable, RaycastResult};
    use aabb::AABB;
    use nalgebra::{Point3, Vector3, Cross, Dot, Norm};
    use rand::{Rng, StdRng, SeedableRng};
    use std::f32::INFINITY;
    use std::cmp;

    type TupleVec = (f32, f32, f32);

    fn tuple_to_point(tpl: &TupleVec) -> Point3<f32> {
        Point3::new(tpl.0, tpl.1, tpl.2)
    }

    fn tuple_to_vector(tpl: &TupleVec) -> Vector3<f32> {
        Vector3::new(tpl.0, tpl.1, tpl.2)
    }

    /// Generates a random `Ray` which points at at a random `AABB`.
    fn gen_ray_to_aabb(data: (TupleVec, TupleVec, TupleVec)) -> (Ray, AABB) {
        // Generate a random AABB
        let mut aabb = AABB::empty()
            .grow(&tuple_to_point(&data.0))
            .grow(&tuple_to_point(&data.1));

        if aabb.surface_area() < EPSILON {
            aabb = aabb.grow(&(aabb.center() + Vector3::new(1.0, 1.0, 1.0)));
        }

        // Get its center
        let center = aabb.center();

        // Generate random ray pointing at the center
        let mut pos = tuple_to_point(&data.2);

        let mut direction = center - pos;
        if direction.norm() < EPSILON {
            pos += Vector3::new(1.0, 0.0, 0.0);
            direction = center - pos;
        }

        (Ray::new(pos, direction), aabb)
    }

    fn is_nan_vector(v: &Vector3<f32>) -> bool {
        v.x != v.x || v.y != v.y || v.z != v.z
    }

    quickcheck!{
        fn test_valid_data_generation(data: (TupleVec, TupleVec, TupleVec)) -> bool {
            let (ray, aabb) = gen_ray_to_aabb(data);
            assert!((ray.direction.norm() - 1.0).abs() < EPSILON);
            assert!(!is_nan_vector(&ray.direction));
            assert!(!is_nan_vector(&ray.inv_direction));

            assert!(aabb.surface_area() > 0.0);
            true
        }
    }

    /// Test the `Intersectable` implementation of AABB.
    /// Only tests if the hit point lies on the AABB's surface.
    quickcheck!{
        fn test_ray_aabb_intersection(data: (TupleVec, TupleVec, TupleVec)) -> bool {
            println!("\n\nNEXT\n");
            let (ray, aabb) = gen_ray_to_aabb(data);

            fn eq_vec(a: TupleVec, b: TupleVec) -> bool {
                (a.0 - b.0).abs() < EPSILON &&
                (a.1 - b.1).abs() < EPSILON &&
                (a.2 - b.2).abs() < EPSILON
            }

            let hit = aabb.intersection(&ray);
            let hit_point = match hit {
                RaycastResult::Hit{ distance } => ray.point_along_direction(hit.distance),
                RaycastResult::Miss => panic!("Raycast missed"),
            };

            // On how many axes the point was on min or max
            let mut matching_axes = 0;
            // How often the point was between min and max instead
            let mut between_min_max = 0;

            fn eq(a: f32, b: f32) -> bool {
                (a - b).abs() < EPSILON
            }

            println!("Ray pos: {:?}", ray.origin);
            println!("Ray dir: {:?}", ray.direction);
            println!("AABB: {:?}", aabb);
            println!("Hit point: {:?}", hit_point);


            for axis in 0..3 {
                if eq(hit_point[axis], aabb.min[axis]) ||
                    eq(hit_point[axis], aabb.max[axis]) {
                        matching_axes += 1;
                    } else if hit_point[axis] > aabb.min[axis] &&
                        hit_point[axis] < aabb.max[axis] {
                        between_min_max += 1;
                    }
            }

            println!("matching_axes {}; between_min_max {}", matching_axes, between_min_max);
            let result = matching_axes >= 1 && matching_axes + between_min_max == 3;
            println!("result: {}\n\nDONE\n", result);
            result
        }
    }

    /// Test whether a `Ray` which points at the center of an `AABB` intersects it.
    /// Uses the optimized algorithm.
    quickcheck!{
        fn test_ray_points_at_aabb_center(data: (TupleVec, TupleVec, TupleVec)) -> bool {
            let (ray, aabb) = gen_ray_to_aabb(data);
            aabb.does_intersect(&ray)
        }
    }

    /// Test whether a `Ray` which points away from the center of an `AABB`
    /// does not intersect it, unless its origin is inside the `AABB`.
    /// Uses the optimized algorithm.
    quickcheck!{
        fn test_ray_points_from_aabb_center(data: (TupleVec, TupleVec, TupleVec)) -> bool {
            let (mut ray, aabb) = gen_ray_to_aabb(data);

            // Invert the direction of the ray
            ray.direction = -ray.direction;
            ray.inv_direction = -ray.inv_direction;
            !aabb.does_intersect(&ray) || aabb.contains(&ray.origin)
        }
    }

    /// Test whether a `Ray` which points at the center of a triangle
    /// intersects it, unless it sees the back face, which is culled.
    quickcheck!{
        fn test_ray_hits_triangle(a: TupleVec,
                                  b: TupleVec,
                                  c: TupleVec,
                                  origin: TupleVec,
                                  u: u16,
                                  v: u16)
                                  -> bool {
            // Define a triangle, u/v vectors and its normal
            let triangle = Triangle::new(tuple_to_point(&a), tuple_to_point(&b), tuple_to_point(&c));
            let u_vec = triangle.b() - triangle.a();
            let v_vec = triangle.c() - triangle.a();
            let normal = u_vec.cross(&v_vec);

            // Get some u and v coordinates such that u+v <= 1
            let u = u % 101;
            let v = cmp::min(100 - u, v % 101);
            let u = u as f32 / 100.0;
            let v = v as f32 / 100.0;

            // Define some point on the triangle
            let point_on_triangle = triangle.a() + u * u_vec + v * v_vec;

            // Define a ray which points at the triangle
            let origin = tuple_to_point(&origin);
            let ray = Ray::new(origin, point_on_triangle - origin);
            let on_back_side = normal.dot(&(ray.origin - triangle.a())) <= 0.0;

            // Perform the intersection test
            let intersects = triangle.does_intersect();
            let uv_sum = intersects.u + intersects.v;

            // Either the intersection is in the back side (including the triangle-plane)
            if on_back_side {
                // Intersection must be INFINITY, u and v are undefined
                intersects.distance == INFINITY
            } else {
                // Or it is on the front side
                // Either the intersection is inside the triangle, which it should be
                // for all u, v such that u+v <= 1.0
                let intersection_inside = uv_sum >= 0.0 && uv_sum <= 1.0 &&
                                          intersects.distance < INFINITY;

                // Or the input data was close to the border
                let close_to_border =
                    u.abs() < EPSILON || (u - 1.0).abs() < EPSILON || v.abs() < EPSILON ||
                    (v - 1.0).abs() < EPSILON || (u + v - 1.0).abs() < EPSILON;

                if !(intersection_inside || close_to_border) {
                    println!("uvsum {}", uv_sum);
                    println!("intersects.0 {}", intersects.distance);
                    println!("intersects.1 (u) {}", intersects.u);
                    println!("intersects.2 (v) {}", intersects.v);
                    println!("u {}", u);
                    println!("v {}", v);
                }

                intersection_inside || close_to_border
            }
        }
    }

    /// Generates some random deterministic `Ray`/`AABB` pairs.
    fn gen_random_ray_aabb(rng: &mut StdRng) -> (Ray, AABB) {
        let a = tuple_to_point(&rng.gen::<TupleVec>());
        let b = tuple_to_point(&rng.gen::<TupleVec>());
        let c = tuple_to_point(&rng.gen::<TupleVec>());
        let d = tuple_to_vector(&rng.gen::<TupleVec>());

        let aabb = AABB::empty().grow(&a).grow(&b);
        let ray = Ray::new(c, d);
        (ray, aabb)
    }

    #[bench]
    /// Benchmark for the optimized intersection algorithm.
    fn bench_intersects_aabb(b: &mut ::test::Bencher) {
        let seed = [0];
        let mut rng = StdRng::from_seed(&seed);

        b.iter(|| {
            let one_thousand = ::test::black_box(1000);
            for _ in 0..one_thousand {
                let (ray, aabb) = gen_random_ray_aabb(&mut rng);
                ray.intersects_aabb(&aabb);
            }
        });
    }

    #[bench]
    /// Benchmark for the naive intersection algorithm.
    fn bench_intersects_aabb_naive(b: &mut ::test::Bencher) {
        let seed = [0];
        let mut rng = StdRng::from_seed(&seed);

        b.iter(|| {
            let one_thousand = ::test::black_box(1000);
            for _ in 0..one_thousand {
                let (ray, aabb) = gen_random_ray_aabb(&mut rng);
                ray.intersects_aabb_naive(&aabb);
            }
        });
    }

    #[bench]
    /// Benchmark for the branchless intersection algorithm.
    fn bench_intersects_aabb_branchless(b: &mut ::test::Bencher) {
        let seed = [0];
        let mut rng = StdRng::from_seed(&seed);

        b.iter(|| {
            let one_thousand = ::test::black_box(1000);
            for _ in 0..one_thousand {
                let (ray, aabb) = gen_random_ray_aabb(&mut rng);
                ray.intersects_aabb_branchless(&aabb);
            }
        });
    }
}
