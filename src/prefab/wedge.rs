use std::ops::{AddAssign, SubAssign};

use num_traits::{cast, Float, FloatConst, NumCast, one, Unsigned, zero};

use crate::polygon::Polygon;
use crate::prefab::polygonal::{Polygonal, PolygonalSides};
use crate::shapes::{Configuration, Shape, Shaper, ShapingError};
use super::linear_algebra::oriented_plane;

/// Right wedges with regular bases and unit distanced apices.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Wedge(Polygon);

impl Wedge {
    /// Create a wedge with `sides` base.
    ///
    /// # Panics
    ///
    /// May panic if `sides` is less than three.
    pub fn new(sides: u16) -> Self {
        Self(Polygon::new(sides))
    }

    /// Convert a [`Polygon`] into a wedge.
    pub fn from_polygon(polygon: Polygon) -> Self {
        Self(polygon)
    }
}

impl<C, I> Polygonal<C, I> for Wedge
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn sides(&self) -> PolygonalSides {
        self.0.sides
    }
}

impl<C, I> Shaper<C, I> for Wedge
where
    C: Float + FloatConst + SubAssign,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn shape(&self, request: Configuration) -> Result<Shape<C, I>, ShapingError> {
        let right_left = !Polygonal::<C, I>::left_right_front_facing(self, request);
        let mut vertices = Polygonal::<C, I>::vertices(self, right_left);
        let radius = Polygonal::<C, I>::radius(self);
        let fh = cast::<_, C>(0.5).unwrap();
        let cos_theta = fh / radius;
        let sin_theta = (C::one() - cos_theta * cos_theta).sqrt();
        let apex = sin_theta * radius * if request.orientation.is_left() {
            -C::one()
        } else {
            one()
        };
        if right_left {
            vertices.push([-fh, zero(), apex]);
            vertices.push([fh, zero(), apex]);
        } else {
            vertices.push([fh, zero(), apex]);
            vertices.push([-fh, zero(), apex]);
        }

        let mut indices = Polygonal::<C, I>::indices(self);
        let mut i = Vec::with_capacity(vertices.len());
        i.push(zero());
        i.push(one());
        for index in 2..vertices.len() {
            i.push(cast::<_, I>(index).ok_or(ShapingError::IndexOverflow)?);
        }

        let base = if request.orientation.is_right() {
            [zero(), zero(), -C::one()]
        } else {
            [zero(), zero(), one()]
        };
        let sides = self.0.sides as usize;
        let mut normals = std::iter::repeat_n(base, sides - 2).collect::<Vec<_>>();

        let offset: usize = sides.is_multiple_of(2).into();
        let vector = [0, 1, sides + offset];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));
        let vector = [0, sides, sides + 1];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));
        for vertex in 2..vertices.len() - 2 {
            let offset: usize = (sides.is_multiple_of(2) != vertex.is_multiple_of(2)).into();
            let vector = [vertex - 2, vertex, sides + offset];
            let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
            normals.push(normal);
            triangle.into_iter().for_each(|index| indices.push(i[index]));
        }
        let vector = [vertices.len() - 4, vertices.len() - 3, sides + offset];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));
        let vector = [vertices.len() - 3 - offset, sides, sides + 1];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));

        if request.generate_normals {
            Shape::with_normals(vertices, normals, indices)
        } else {
            Shape::without_normals(vertices, indices)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Wedge, Shape, Shaper};

    use super::super::{
        linear_algebra::magnitude_squared,
        unit_test::epsilon_error,
    };

    fn make_shape(size: u16) -> Shape<f64, u16> {
        Wedge::new(size).shape(Default::default()).expect("Panics occur before this call")
    }

    #[test]
    fn right_apices_dot() {
        let shape = make_shape(4);
        let vertices = shape.vertices();

        let a = vertices[0];
        let b = vertices[vertices.len() - 4];
        epsilon_error((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).abs());

        let a = vertices[1];
        let b = vertices[vertices.len() - 3];
        epsilon_error((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]).abs());
    }

    #[test]
    fn right_apices_pythagoras() {
        let shape = make_shape(u16::MAX - 1);
        let vertices = shape.vertices();

        let a = vertices[0];
        let b = vertices[vertices.len() - 4];
        let c = vertices[vertices.len() - 2];
        epsilon_error(
            magnitude_squared([a[0] - b[0], a[1] - b[1], a[2] - b[2]]) -
            magnitude_squared([a[0] - c[0], a[1] - c[1], a[2] - c[2]]) -
            magnitude_squared([c[0] - b[0], c[1] - b[1], c[2] - b[2]]));

        let a = vertices[1];
        let b = vertices[vertices.len() - 3];
        let c = vertices[vertices.len() - 1];
        epsilon_error(
            magnitude_squared([a[0] - b[0], a[1] - b[1], a[2] - b[2]]) -
            magnitude_squared([a[0] - c[0], a[1] - c[1], a[2] - c[2]]) -
            magnitude_squared([c[0] - b[0], c[1] - b[1], c[2] - b[2]]));
    }
}

