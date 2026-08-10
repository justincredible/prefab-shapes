use std::ops::{AddAssign, SubAssign};

use num_traits::{cast, Float, FloatConst, NumCast, one, Unsigned, zero};

use crate::polygon::Polygon;
use crate::prefab::polygonal::{Polygonal, PolygonalSides};
use crate::shapes::{Configuration, Shape, Shaper, ShapingError};
use super::linear_algebra::oriented_plane;

/// Right<sup>2</sup> pyramids with regular bases.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Pyramid(Polygon);

impl Pyramid {
    /// Create a pyramid with `sides` base.
    ///
    /// # Panics
    ///
    /// May panic if `sides` is less than three.
    pub fn new(sides: u16) -> Self {
        Self(Polygon::new(sides))
    }

    /// Convert a [`Polygon`] into a pyramid.
    pub fn from_polygon(polygon: Polygon) -> Self {
        Self(polygon)
    }
}

impl<C, I> Polygonal<C, I> for Pyramid
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn sides(&self) -> PolygonalSides {
        self.0.sides
    }
}

impl<C, I> Shaper<C, I> for Pyramid
where
    C: Float + FloatConst + SubAssign,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn shape(&self, request: Configuration) -> Result<Shape<C, I>, ShapingError> {
        let right_left = !Polygonal::<C, I>::left_right_front_facing(self, request);
        let mut vertices = Polygonal::<C, I>::vertices(self, right_left);
        let apex = Polygonal::<C, I>::radius(self) * if request.orientation.is_left() {
            -C::one()
        } else {
            one()
        };
        vertices.push([zero(), zero(), apex]);

        let mut indices = Polygonal::<C, I>::indices(self);
        let i = vec![zero(), one()]
            .into_iter()
            .chain((2..vertices.len()).map(|i| cast::<_, I>(i).unwrap()))
            .collect::<Vec<_>>();
        let ordered_orientation = right_left != request.orientation.is_ccw();
        if ordered_orientation == self.0.sides.is_multiple_of(2) {
            indices.push(i[0]);
            indices.push(i[1]);
        } else {
            indices.push(i[1]);
            indices.push(i[0]);
        }
        indices.push(i[self.0.sides as usize]);
        for vertex_index in 2..vertices.len() - 1 {
            if ordered_orientation == (self.0.sides as usize + vertex_index).is_multiple_of(2) {
                indices.push(i[vertex_index]);
                indices.push(i[vertex_index - 2]);
            } else {
                indices.push(i[vertex_index - 2]);
                indices.push(i[vertex_index]);
            }
            indices.push(i[self.0.sides as usize]);
        }
        if ordered_orientation {
            indices.push(i[vertices.len() - 2]);
            indices.push(i[vertices.len() - 3]);
        } else {
            indices.push(i[vertices.len() - 3]);
            indices.push(i[vertices.len() - 2]);
        }
        indices.push(i[self.0.sides as usize]);

        if request.generate_normals {
            let base = if request.orientation.is_right() {
                [zero(), zero(), -C::one()]
            } else {
                [zero(), zero(), one()]
            };
            let polygonal_triangles = self.0.sides as usize - 2;
            let mut normals = std::iter::repeat(base).take(polygonal_triangles).collect::<Vec<_>>();
            for triple in indices.chunks(3).skip(polygonal_triangles) {
                let face = triple.into_iter().map(|&i| i.to_usize().unwrap()).collect::<Vec<_>>();
                let (normal, _triangle) = oriented_plane(&vertices, &face, request.orientation);
                normals.push(normal);
            }

            Shape::with_normals(vertices, normals, indices)
        } else {
            Shape::without_normals(vertices, indices)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Pyramid, Shape, Shaper};

    use super::super::{
        linear_algebra::magnitude_diff,
        unit_test::{distance_neighbour, epsilon_error},
    };

    fn make_shape(size: u16) -> Shape<f64, u16> {
        Pyramid::new(size).shape(Default::default()).expect("Panics occur before this call")
    }

    #[test]
    fn right_center_odd_lowest() {
        let shape = make_shape(3);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        for index in 1..vertices.len() - 1 {
            distance_neighbour(distance, vertices, index, vertices.len() - 1);
        }
    }

    #[test]
    fn right_center_even_lowest() {
        let shape = make_shape(4);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        for index in 1..vertices.len() - 1 {
            distance_neighbour(distance, vertices, index, vertices.len() - 1);
        }
    }

    #[test]
    fn right_center_odd_higher() {
        let shape = make_shape(17);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        for index in 1..vertices.len() - 1 {
            distance_neighbour(distance, vertices, index, vertices.len() - 1);
        }
    }

    #[test]
    fn right_center_even_higher() {
        let shape = make_shape(16);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        for index in 1..vertices.len() - 1 {
            distance_neighbour(distance, vertices, index, vertices.len() - 1);
        }
    }

    #[test]
    fn right_center_error_total_odd() {
        let shape = make_shape(32773);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        let mut error = 0.;
        for i in 1..vertices.len() - 1 {
            error += distance - magnitude_diff(vertices[i], vertices[vertices.len() - 1]);
        }

        epsilon_error(error);
    }

    #[test]
    fn right_center_error_total_even() {
        let shape = make_shape(32768);
        let vertices = shape.vertices();
        let distance = magnitude_diff(vertices[0], vertices[vertices.len() - 1]);

        let mut error = 0.;
        for i in 1..vertices.len() - 1 {
            error += distance - magnitude_diff(vertices[i], vertices[vertices.len() - 1]);
        }

        epsilon_error(error);
    }

    #[test]
    fn right_diameter_first() {
        let shape = make_shape(u16::MAX - 1);
        let vertices = shape.vertices();

        let a = vertices[0];
        let b = vertices[vertices.len() - 1];
        epsilon_error((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).abs());
    }

    #[test]
    fn right_diameter_lowest() {
        let shape = make_shape(4);
        let vertices = shape.vertices();

        for i in 0..vertices.len() / 2 {
            let a = vertices[i];
            let b = vertices[vertices.len() - 1 - i];
            epsilon_error((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).abs());
        }
    }

    #[test]
    fn right_diameter_error_total() {
        let shape = make_shape(u16::MAX / 2);
        let vertices = shape.vertices();

        let mut error = 0.;
        for i in 0..vertices.len() / 2 {
            let a = vertices[i];
            let b = vertices[vertices.len() - 1 - i];
            error += a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
        }

        epsilon_error(error);
    }
}
