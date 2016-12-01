//! TODO comment
//!

use EPSILON;
use bounding_hierarchy::BoundingHierarchy;
use axis::Axis;
use aabb::{AABB, Bounded};
// use aap::AAP;
use ray::Ray;
use std::boxed::Box;
use std::f32;
use std::iter::repeat;

/// TODO comment
///
pub enum BIHNode {
    /// Leaf node.
    Leaf {
        /// The index of the shape contained in this leaf.
        shapes: Vec<usize>,
    },
    /// Inner node.
    Node {
        /// The split axis.
        split_axis: Axis,

        /// The "left" plane splitting child_l from the rest.
        child_l_plane: f32,

        /// Left subtree.
        child_l: Box<BIHNode>,

        /// The "right" plane splitting child_r from the rest.
        child_r_plane: f32,

        /// Left subtree.
        child_r: Box<BIHNode>,
    },
}

impl BIHNode {
    /// Builds a [`BIHNode`] recursively using SAH partitioning.
    ///
    /// [`BIHNode`]: enum.BIHNode.html
    ///
    pub fn build<T: Bounded>(shapes: &[T], indices: Vec<usize>) -> (AABB, BIHNode) {
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

        // If there is five or fewer elements, don't split anymore
        if indices.len() <= 5 {
            return (aabb_bounds, BIHNode::Leaf { shapes: indices });
        }

        // Find the axis along which the shapes are spread the most
        let split_axis = centroid_bounds.largest_axis();
        let split_axis_size = centroid_bounds.max[split_axis] - centroid_bounds.min[split_axis];

        if split_axis_size < EPSILON {
            return (aabb_bounds, BIHNode::Leaf { shapes: indices });
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

        let (_, node_l) = BIHNode::build(shapes, child_l_indices);
        let (_, node_r) = BIHNode::build(shapes, child_r_indices);

        // Construct the actual data structure
        let node = BIHNode::Node {
            split_axis: split_axis,
            child_l_plane: child_l_aabb.max[split_axis],
            child_l: Box::new(node_l),
            child_r_plane: child_r_aabb.min[split_axis],
            child_r: Box::new(node_r),
        };

        (aabb_bounds, node)
    }

    /// Prints a textual representation of the recursive [`BIH`] structure.
    ///
    /// [`BIH`]: struct.BIH.html
    ///
    fn pretty_print(&self, depth: usize) {
        let padding: String = repeat(" ").take(depth).collect();
        let axes = ["X", "Y", "Z"];
        match *self {
            BIHNode::Node { ref split_axis,
                            ref child_l_plane,
                            ref child_l,
                            ref child_r_plane,
                            ref child_r } => {
                println!("{}child_l ({}, {})",
                         padding,
                         axes[*split_axis as usize],
                         child_l_plane);
                child_l.pretty_print(depth + 1);
                println!("{}child_r ({}, {})",
                         padding,
                         axes[*split_axis as usize],
                         child_r_plane);
                child_r.pretty_print(depth + 1);
            }
            BIHNode::Leaf { ref shapes } => {
                println!("{}shapes\t{:?}", padding, shapes);
            }
        }
    }

    /// Traverses the [`BIH`] recursively and insterts shapes which are hit with a
    /// high probability by `ray` into the [`Vec`] `indices`.
    ///
    /// [`BIH`]: struct.BIH.html
    /// [`Vec`]: https://doc.rust-lang.org/std/vec/struct.Vec.html
    ///
    pub fn traverse_recursive(&self, ray: &Ray, max_distance: f32, indices: &mut Vec<usize>) {
        match *self {
            BIHNode::Node { ref split_axis,
                            ref child_l_plane,
                            ref child_l,
                            ref child_r_plane,
                            ref child_r } => {
                let origin = ray.origin[*split_axis];
                let direction = ray.direction[*split_axis];
                let direction_goes_right = direction >= 0.0;
                // Distance between origin and planes
                let origin_distance_l = child_l_plane - origin;
                let origin_distance_r = child_r_plane - origin;

                if origin < *child_l_plane {
                    // Ray starts inside left side
                    // print!("left-{} ", *split_axis);
                    // print!("left ");

                    let new_max_distance = if direction_goes_right {
                        (origin_distance_l / direction).abs()
                    } else {
                        max_distance
                    };
                    child_l.traverse_recursive(ray, new_max_distance, indices);
                } else if !direction_goes_right &&
                          origin_distance_l <= max_distance * direction.abs() {
                    // Ray enters left side
                    // print!("left&-{} ", *split_axis);
                    // print!("left ");

                    let origin_delta = origin_distance_l / direction;
                    let new_ray = Ray::new_with_moved_origin(ray, origin_delta);
                    let new_max_distance = max_distance - origin_delta.abs();
                    child_l.traverse_recursive(&new_ray, new_max_distance, indices);
                }

                if origin > *child_r_plane {
                    // Ray starts inside right side
                    // print!("right-{} ", *split_axis);
                    // print!("right ");

                    let new_max_distance = if !direction_goes_right {
                        (origin_distance_r / direction).abs()
                    } else {
                        max_distance
                    };
                    child_r.traverse_recursive(ray, new_max_distance, indices);
                } else if direction_goes_right &&
                          origin_distance_r <= max_distance * direction.abs() {
                    // Ray enters right side
                    // print!("right&-{} ", *split_axis);
                    // print!("right ");

                    let origin_delta = origin_distance_r / direction;
                    let new_ray = Ray::new_with_moved_origin(ray, origin_delta);
                    let new_max_distance = max_distance - origin_delta.abs();
                    child_r.traverse_recursive(&new_ray, new_max_distance, indices);
                }

                // print!("END ");
            }
            BIHNode::Leaf { ref shapes } => {
                // print!("LEAF ");
                for index in shapes {
                    indices.push(*index);
                }
            }
        }
    }

    // /// TODO proper comment
    // /// Gets the index of the first shape that the passed function considers intersected.
    // /// This function benefits from the fact that when using BIHs,
    // /// we know which child node a ray traverses first.
    // ///
    // pub fn get_first_intersection<'a, F>(&self, ray: &Ray, intersection_checker: &F) -> usize
    //     where F: Fn(&Ray, usize) -> bool {
    //     struct TraversalBranch<'a> {
    //         node: &'a BIHNode,
    //         aap: AAP,
    //     }
    //     // Inner recursive function
    //     fn get_first_intersection_inner<'a>(node: &'a BIHNode, ray: &'a Ray, stack: &'a mut Vec<TraversalBranch<'a>>) -> usize {
    //         match *node {
    //             BIHNode::Node { ref split_axis, ref child_l_plane, ref child_l, ref child_r_plane, ref child_r } => {
    //                 let mut result = usize::max_value();
    //
    //                 let axis = *split_axis as usize;
    //                 let origin = ray.origin[axis];
    //                 let direction = ray.direction[axis];
    //                 if direction == 0.0 {
    //                     // Check both sides unbiased (how??)
    //                 } else {
    //                     let left_plane = AAP::new(axis, *child_l_plane);
    //                     let right_plane = AAP::new(axis, *child_r_plane);
    //                     // If the ray goes right...
    //                     if direction > 0.0 {
    //                         // ...check the left volume first
    //                         if origin < *child_l_plane {
    //                             // Put the right side on the stack for later
    //                             stack.push(TraversalBranch{node: child_l, aap: left_plane});
    //                             // Traverse the left side
    //
    //                         } else {
    //                             // Traverse the right side
    //                         }
    //                     } else {
    //                         // Otherwise, check the right volume first
    //                         if origin > *child_r_plane {
    //                             // Put the left side on the stack for later
    //                             // Traverse the right side
    //                         } else {
    //                             // Traverse the left side
    //                         }
    //                     }
    //                 }
    //                 result
    //             }
    //             BIHNode::Leaf { ref shape } => {
    //                 if |ray, shape|{ intersection_checker(&ray, *shape) } { *shape } else { usize::max_value() }
    //             }
    //         }
    //     }
    //
    //     let mut stack = Vec::new();
    //     get_first_intersection_inner(&self, ray, &mut stack)
    // }
}

/// TODO comment
///
pub struct BIH {
    /// TODO comment
    pub aabb: AABB,

    /// TODO comment
    pub root: BIHNode,
}

impl BIH {
    /// Creates a new [`BIH`] from the `shapes` slice.
    ///
    /// [`BIH`]: struct.BIH.html
    ///
    /// # Examples
    ///
    /// // TODO example
    ///
    pub fn build<T: Bounded>(shapes: &[T]) -> BIH {
        let indices = (0..shapes.len()).collect::<Vec<usize>>();
        let (aabb, root) = BIHNode::build(shapes, indices);
        BIH {
            aabb: aabb,
            root: root,
        }
    }
}

impl BoundingHierarchy for BIH {
    fn build<T: Bounded>(shapes: &[T]) -> BIH {
        BIH::build(shapes)
    }

    fn traverse<'a, T: Bounded>(&'a self, ray: &Ray, shapes: &'a [T]) -> Vec<&T> {
        // println!("BIH BIH BIH BIH");
        let mut hit_shapes = Vec::new();

        if ray.intersects_aabb(&self.aabb) {
            let max_distance = ray.distance_to_aabb_end(&self.aabb);
            let mut indices = Vec::new();

            self.root.traverse_recursive(ray, max_distance, &mut indices);

            for index in &indices {
                let shape = &shapes[*index];
                if ray.intersects_aabb(&shape.aabb()) {
                    hit_shapes.push(shape);
                }
            }
        }
        // println!("");
        // println!("BIH BIH BIH BIH");
        hit_shapes
    }

    fn pretty_print(&self) {
        self.root.pretty_print(0);
    }
}

#[cfg(test)]
pub mod tests {
    use bounding_hierarchy::BoundingHierarchy;
    use bih::BIH;
    use testbase::*;

    /// Creates a `BIH` for a fixed scene structure.
    pub fn build_some_bih() -> (Vec<UnitBox>, BIH) {
        let shapes = generate_aligned_boxes();

        let bih = BoundingHierarchy::build(&shapes);
        (shapes, bih)
    }

    #[test]
    /// Tests whether the building procedure succeeds in not failing.
    fn test_build_bih_aligned_boxes() {
        build_some_bih();
    }

    #[test]
    /// Runs some primitive tests for intersections of a ray with a fixed scene given as a BIH.
    fn test_traverse_bih() {
        let (shapes, bih) = build_some_bih();

        test_traverse_aligned_boxes(&bih, &shapes);
    }

    // #[bench]
    // /// Benchmark the construction of a BIH with 1,200 triangles.
    // fn bench_build_48_triangles_bih(mut b: &mut ::test::Bencher) {
    //     bench_intersect_n_triangles::<BIH>(4, &mut b);
    // }

    #[bench]
    /// Benchmark the construction of a BIH with 1,200 triangles.
    fn bench_build_1200_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_build_1200_triangles::<BIH>(&mut b);
    }

    #[bench]
    /// Benchmark the construction of a BIH with 12,000 triangles.
    fn bench_build_12k_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_build_12k_triangles::<BIH>(&mut b);
    }

    #[bench]
    /// Benchmark the construction of a BIH with 120,000 triangles.
    fn bench_build_120k_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_build_120k_triangles::<BIH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 1,200 triangles using the recursive BIH.
    fn bench_intersect_1200_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_intersect_1200_triangles::<BIH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 12,000 triangles using the recursive BIH.
    fn bench_intersect_12k_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_intersect_12k_triangles::<BIH>(&mut b);
    }

    #[bench]
    /// Benchmark intersecting 120,000 triangles using the recursive BIH.
    fn bench_intersect_120k_triangles_bih(mut b: &mut ::test::Bencher) {
        bench_intersect_120k_triangles::<BIH>(&mut b);
    }
}
