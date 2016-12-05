//! This module defines a [`BVH`] building procedure as well as a [`BVH`] flattening procedure
//! so that the recursive structure can be easily used in compute shaders.
//!
//! [`BVH`]: struct.BVH.html
//!

use EPSILON;
use bounding_hierarchy::BoundingHierarchy;
use aabb::{AABB, Bounded};
use ray::Ray;
use raycast::Intersectable;
use std::boxed::Box;
use std::f32;
use std::iter::repeat;

/// Enum which describes the union type of a node in a [`BVH`].
/// This structure does not allow for storing a root node's [`AABB`]. Therefore rays
/// which do not hit the root [`AABB`] perform two [`AABB`] tests (left/right) instead of one.
/// On the other hand this structure decreases the total number of indirections when traversing
/// the BVH. Only those nodes are accessed, which are definetely hit.
///
/// [`AABB`]: ../aabb/struct.AABB.html
/// [`BVH`]: struct.BVH.html
///
pub enum BVHNode {
    /// Leaf node.
    Leaf {
        /// The shapes contained in this leaf.
        shapes: Vec<usize>,
    },
    /// Inner node.
    Node {
        /// The convex hull of the shapes' `AABB`s in child_l.
        child_l_aabb: AABB,

        /// Left subtree.
        child_l: Box<BVHNode>,

        /// The convex hull of the shapes' `AABB`s in child_r.
        child_r_aabb: AABB,

        /// Left subtree.
        child_r: Box<BVHNode>,
    },
}

impl BVHNode {
    /// Builds a [`BVHNode`] recursively using SAH partitioning.
    ///
    /// [`BVHNode`]: enum.BVHNode.html
    ///
    pub fn build<T: Bounded>(shapes: &[T], indices: Vec<usize>) -> BVHNode {
        // Helper function to accumulate the AABB joint and the centroids AABB
        fn grow_convex_hull(convex_hull: (AABB, AABB), shape_aabb: &AABB) -> (AABB, AABB) {
            let center = &shape_aabb.center();
            let convex_hull_aabbs = &convex_hull.0;
            let convex_hull_centroids = &convex_hull.1;
            (convex_hull_aabbs.join(shape_aabb), convex_hull_centroids.grow(center))
        }

        let mut convex_hull = Default::default();
        for index in &indices {
            convex_hull = grow_convex_hull(convex_hull, &shapes[*index].aabb());
        }
        let (aabb_bounds, centroid_bounds) = convex_hull;

        // If there are five or fewer elements, don't split anymore
        if indices.len() <= 5 {
            return BVHNode::Leaf { shapes: indices };
        }

        // Find the axis along which the shapes are spread the most
        let split_axis = centroid_bounds.largest_axis();
        let split_axis_size = centroid_bounds.max[split_axis] - centroid_bounds.min[split_axis];

        if split_axis_size < EPSILON {
            return BVHNode::Leaf { shapes: indices };
        }

        /// Defines a Bucket utility object.
        #[derive(Copy, Clone)]
        struct Bucket {
            size: usize,
            aabb: AABB,
        }

        impl Bucket {
            /// Returns an empty bucket.
            fn empty() -> Bucket {
                Bucket {
                    size: 0,
                    aabb: AABB::empty(),
                }
            }

            /// Extends this `Bucket` by the given `AABB`.
            fn add_aabb(&mut self, aabb: &AABB) {
                self.size += 1;
                self.aabb = self.aabb.join(aabb);
            }
        }

        /// Joins two `Bucket`s.
        fn join_bucket(a: Bucket, b: &Bucket) -> Bucket {
            Bucket {
                size: a.size + b.size,
                aabb: a.aabb.join(&b.aabb),
            }
        }

        // Create six buckets, and six index assignment vectors
        const NUM_BUCKETS: usize = 6;
        let mut buckets = [Bucket::empty(); NUM_BUCKETS];
        let mut bucket_assignments: [Vec<usize>; NUM_BUCKETS] = Default::default();

        // Assign each shape to a bucket
        for idx in &indices {
            let shape = &shapes[*idx];
            let shape_aabb = shape.aabb();
            let shape_center = shape_aabb.center();

            // Get the relative position of the shape centroid [0.0..1.0]
            let bucket_num_relative = (shape_center[split_axis] - centroid_bounds.min[split_axis]) /
                                      split_axis_size;

            // Convert that to the actual `Bucket` number
            let bucket_num = (bucket_num_relative * (NUM_BUCKETS as f32 - 0.01)) as usize;

            // Extend the selected `Bucket` and add the index to the actual bucket
            buckets[bucket_num].add_aabb(&shape_aabb);
            bucket_assignments[bucket_num].push(*idx);
        }

        // Compute the costs for each configuration and
        // select the configuration with the minimal costs
        let mut min_bucket = 0;
        let mut min_cost = f32::INFINITY;
        let mut child_l_aabb = AABB::empty();
        let mut child_r_aabb = AABB::empty();
        for i in 0..(NUM_BUCKETS - 1) {
            let child_l = buckets.iter().take(i + 1).fold(Bucket::empty(), join_bucket);
            let child_r = buckets.iter().skip(i + 1).fold(Bucket::empty(), join_bucket);

            let cost = (child_l.size as f32 * child_l.aabb.surface_area() +
                        child_r.size as f32 * child_r.aabb.surface_area()) /
                       aabb_bounds.surface_area();

            if cost < min_cost {
                min_bucket = i;
                min_cost = cost;
                child_l_aabb = child_l.aabb;
                child_r_aabb = child_r.aabb;
            }
        }

        // Join together all index buckets, and proceed recursively
        let mut child_l_indices = Vec::new();
        for mut indices in bucket_assignments.iter_mut().take(min_bucket + 1) {
            child_l_indices.append(&mut indices);
        }
        let mut child_r_indices = Vec::new();
        for mut indices in bucket_assignments.iter_mut().skip(min_bucket + 1) {
            child_r_indices.append(&mut indices);
        }

        // Construct the actual data structure
        BVHNode::Node {
            child_l_aabb: child_l_aabb,
            child_l: Box::new(BVHNode::build(shapes, child_l_indices)),
            child_r_aabb: child_r_aabb,
            child_r: Box::new(BVHNode::build(shapes, child_r_indices)),
        }
    }

    /// Prints a textual representation of the recursive [`BVH`] structure.
    ///
    /// [`BVH`]: struct.BVH.html
    ///
    fn pretty_print(&self, depth: usize) {
        let padding: String = repeat(" ").take(depth).collect();
        match *self {
            BVHNode::Node { ref child_l, ref child_r, ref child_l_aabb, ref child_r_aabb } => {
                println!("{}child_l ({})", padding, child_l_aabb);
                child_l.pretty_print(depth + 1);
                println!("{}child_r ({})", padding, child_r_aabb);
                child_r.pretty_print(depth + 1);
            }
            BVHNode::Leaf { ref shapes } => {
                println!("{}shapes\t{:?}", padding, shapes);
            }
        }
    }

    /// Traverses the [`BVH`] recursively and insterts shapes which are hit with a
    /// high probability by `ray` into the [`Vec`] `indices`.
    ///
    /// [`BVH`]: struct.BVH.html
    /// [`Vec`]: https://doc.rust-lang.org/std/vec/struct.Vec.html
    ///
    pub fn traverse_recursive(&self, ray: &Ray, indices: &mut Vec<usize>) {
        match *self {
            BVHNode::Node { ref child_l_aabb, ref child_l, ref child_r_aabb, ref child_r } => {
                if child_l_aabb.does_intersect(ray) {
                    // print!("left ");
                    child_l.traverse_recursive(ray, indices);
                }
                if child_r_aabb.does_intersect(ray) {
                    // print!("right ");
                    child_r.traverse_recursive(ray, indices);
                }
                // print!("END ");
            }
            BVHNode::Leaf { ref shapes } => {
                // print!("LEAF ");
                for index in shapes {
                    indices.push(*index);
                }
            }
        }
    }
}

/// The [`BVH`] data structure. Only contains the root node of the [`BVH`] tree.
///
/// [`BVH`]: struct.BVH.html
/// [`build`]: struct.BVH.html#method.build
///
pub struct BVH {
    /// The root node of the [`BVH`].
    ///
    /// [`BVH`]: struct.BVH.html
    ///
    pub root: BVHNode,
}

impl BVH {
    /// Creates a new [`BVH`] from the `shapes` slice.
    ///
    /// [`BVH`]: struct.BVH.html
    ///
    /// # Examples
    ///
    /// // TODO example
    ///
    pub fn build<T: Bounded>(shapes: &[T]) -> BVH {
        let indices = (0..shapes.len()).collect::<Vec<usize>>();
        let root = BVHNode::build(shapes, indices);
        BVH { root: root }
    }
}

impl BoundingHierarchy for BVH {
    fn build<T: Bounded>(shapes: &[T]) -> BVH {
        BVH::build(shapes)
    }

    fn traverse<'a, T: Bounded>(&'a self, ray: &Ray, shapes: &'a [T]) -> Vec<&T> {
        // println!("BVH BVH BVH BVH");
        let mut indices = Vec::new();
        self.root.traverse_recursive(ray, &mut indices);
        let mut hit_shapes = Vec::new();
        for index in &indices {
            let shape = &shapes[*index];
            if shape.aabb().does_intersect(ray) {
                hit_shapes.push(shape);
            }
        }
        // println!("");
        // println!("BVH BVH BVH BVH");
        hit_shapes
    }

    fn pretty_print(&self) {
        self.root.pretty_print(0);
    }
}

#[cfg(test)]
pub mod tests {
    use bounding_hierarchy::BoundingHierarchy;
    use bvh::BVH;
    use testbase::*;

    /// Creates a `BVH` for a fixed scene structure.
    pub fn build_some_bvh() -> (Vec<UnitBox>, BVH) {
        let shapes = generate_aligned_boxes();

        let bvh = BoundingHierarchy::build(&shapes);
        (shapes, bvh)
    }

    #[test]
    /// Tests whether the building procedure succeeds in not failing.
    fn test_build_bvh_aligned_boxes() {
        build_some_bvh();
    }

    #[test]
    /// Runs some primitive tests for intersections of a ray with a fixed scene given as a `BVH`.
    fn test_traverse_bvh() {
        let (shapes, bvh) = build_some_bvh();

        test_traverse_aligned_boxes(&bvh, &shapes);
    }

    // #[bench]
    // /// Benchmark the construction of a BIH with 1,200 triangles.
    // fn bench_build_48_triangles_bvh(mut b: &mut ::test::Bencher) {
    //     bench_intersect_n_triangles::<BVH>(4, &mut b);
    // }

    #[bench]
    /// Benchmark the construction of a `BVH` with 1,200 triangles.
    fn bench_build_1200_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_build_1200_triangles::<BVH>(&mut b);
    }

    #[bench]
    /// Benchmark the construction of a `BVH` with 12,000 triangles.
    fn bench_build_12k_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_build_12k_triangles::<BVH>(&mut b);
    }

    #[bench]
    /// Benchmark the construction of a `BVH` with 120,000 triangles.
    fn bench_build_120k_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_build_120k_triangles::<BVH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 1,200 triangles using the recursive `BVH`.
    fn bench_intersect_1200_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_intersect_1200_triangles::<BVH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 12,000 triangles using the recursive `BVH`.
    fn bench_intersect_12k_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_intersect_12k_triangles::<BVH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 120,000 triangles using the recursive `BVH`.
    fn bench_intersect_120k_triangles_bvh(mut b: &mut ::test::Bencher) {
        bench_intersect_120k_triangles::<BVH>(&mut b);
    }
}
