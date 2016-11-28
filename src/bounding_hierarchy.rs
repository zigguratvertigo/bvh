//! This module defines the `BoundingHierarchy` trait.

use aabb::Bounded;
use ray::Ray;

pub trait BoundingHierarchy {
    /// Creates a new [`BIH`] from the `shapes` slice.
    ///
    /// [`BIH`]: struct.BIH.html
    ///
    /// # Examples
    ///
    /// TODO example
    ///
    fn build<T: Bounded>(shapes: &[T]) -> Self;

    /// Traverses the tree recursively. Returns a subset of `shapes`, in which the [`AABB`]s
    /// of the elements were hit by the `ray`.
    ///
    /// [`AABB`]: ../aabb/struct.AABB.html
    ///
    /// # Examples
    ///
    /// TODO example
    ///
    fn traverse<'a, T: Bounded>(&'a self, ray: &Ray, shapes: &'a [T]) -> Vec<&T>;

    /// Prints the [`BoundingHierarchy`] in a tree-like visualization.
    ///
    /// // TODO does this even
    /// [`BoundingHierarchy`]: trait.BoundingHierarchy.html
    ///
    fn pretty_print(&self);
}
