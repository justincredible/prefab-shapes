use std::ops::{AddAssign, SubAssign};

use num_traits::{cast, Float, FloatConst, NumCast, one, Unsigned, zero};

use crate::polygon::Polygon;
use crate::prefab::polygonal::{Polygonal, PolygonalSides};
use crate::shapes::{Configuration, Shape, Shaper, ShapingError};
use super::linear_algebra::{oriented_plane, rotation_z};

/// Right prisms with regular bases.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Prism {
    anti: bool,
    polygon: Polygon
}

impl Prism {
    /// Create a prism with `sides` base.
    ///
    /// # Panics
    ///
    /// May panic if `sides` is less than three.
    pub fn new(sides: u16, anti: bool) -> Self {
        Self { anti, polygon: Polygon::new(sides) }
    }

    /// Convert a [`Polygon`] into a prism.
    pub fn from_polygon(polygon: Polygon, anti: bool) -> Self {
        Self { anti, polygon }
    }
}

impl<C, I> Polygonal<C, I> for Prism
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn sides(&self) -> PolygonalSides {
        self.polygon.sides
    }
}

impl<C, I> Shaper<C, I> for Prism
where
    C: Float + FloatConst + AddAssign + SubAssign,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn shape(&self, request: Configuration) -> Result<Shape<C, I>, ShapingError> {
        let fh = cast::<_, C>(0.5).unwrap();
        let mut half_height = if self.anti {
            let f1 = C::one();
            let f2 = f1 + f1;
            let r = Polygonal::<C, I>::radius(self);
            let rr = r * r;
            fh * (f1 - rr * (f2 - (f2 + f2 - f1 / rr).sqrt())).sqrt()
        } else {
            fh
        };
        if request.orientation.is_left() {
            half_height = -half_height;
        }
        let (mut half_angle, _) = Polygonal::<C, I>::angle(self);
        if request.orientation.is_cw() {
            half_angle = -half_angle;
        }
        let left_right = Polygonal::<C, I>::left_right_front_facing(self, request);
        let mut vertices = Polygonal::<C, I>::vertices(self, !left_right);
        for vertex in &mut vertices {
            vertex[2] -= half_height;
            if self.anti {
                rotation_z(vertex, half_angle);
            }
        }
        for mut vertex in Polygonal::<C, I>::vertices(self, left_right) {
            vertex[2] += half_height;
            vertices.push(vertex);
        }
        let mut indices = Polygonal::<C, I>::indices(self);
        let sides = self.polygon.sides as usize;
        let base = cast::<_, I>(sides).ok_or(ShapingError::IndexOverflow)?;
        // check all vertices do not exceed index types maximum
        cast::<_, I>(vertices.len() - 1).ok_or(ShapingError::IndexOverflow)?;
        for index in Polygonal::<C, I>::indices(self) {
            indices.push(index + base);
        }
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
        let mut top = base;
        top[2] = -top[2];
        let mut normals = std::iter::repeat_n(base, sides - 2)
            .chain(std::iter::repeat_n(top, sides - 2))
            .collect::<Vec<_>>();

        let offset = sides.is_multiple_of(2).into();
        let vector = [0, 1, sides + offset];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));
        let vector = [offset, sides, sides + 1];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));

        for vertex in 2..sides {
            let offset = if !sides.is_multiple_of(2) {
                vertex - 1
            } else if vertex.is_multiple_of(2) {
                vertex + 1
            } else {
                vertex - 3
            };
            let vector = [vertex - 2, vertex, sides + offset];
            let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
            normals.push(normal);
            triangle.into_iter().for_each(|index| indices.push(i[index]));
            let vector = [offset, sides + vertex - 2, sides + vertex];
            let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
            normals.push(normal);
            triangle.into_iter().for_each(|index| indices.push(i[index]));
        }

        let vector = [sides - 2, sides - 1, 2 * sides - 1 - offset];
        let (normal, triangle) = oriented_plane(&vertices, &vector, request.orientation);
        normals.push(normal);
        triangle.into_iter().for_each(|index| indices.push(i[index]));
        let vector = [sides - 1 - offset, 2 * sides - 2, 2 * sides - 1];
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
    use super::{Prism, Shape, Shaper};

    use super::super::unit_test::distance_neighbour;

    fn make_shape(size: u16, anti: bool) -> Shape<f64, u16> {
        Prism::new(size, anti).shape(Default::default()).expect("Panics occur before this call")
    }

    #[test]
    fn prism_ring_edges_odd() {
        let sides = u16::MAX / 2;
        let shape = make_shape(sides, false);
        let vertices = shape.vertices();
        let sides = sides as usize;

        distance_neighbour(1., vertices, 0, sides);
        for index in 1..sides {
            let offset = if !index.is_multiple_of(2) {
                index + 1
            } else {
                index - 1
            };
            distance_neighbour(1., vertices, index, sides + offset);
        }
    }

    #[test]
    fn prism_ring_edges_even() {
        let sides = u16::MAX / 2 + 1;
        let shape = make_shape(sides, false);
        let vertices = shape.vertices();
        let sides = sides as usize;

        for index in 0..sides {
            let offset = if index.is_multiple_of(2) {
                index + 1
            } else {
                index - 1
            };
            distance_neighbour(1., vertices, index, sides + offset);
        }
    }

    #[test]
    fn antiprism_ring_edges_odd() {
        let sides = 11;
        let shape = make_shape(sides, true);
        let vertices = shape.vertices();
        let sides = sides as usize;

        distance_neighbour(1., vertices, 0, sides);
        distance_neighbour(1., vertices, 0, sides + 1);
        distance_neighbour(1., vertices, 1, sides);
        for index in 2..sides {
            let offset = index - 1;
            distance_neighbour(1., vertices, offset, sides + index - 2);
            distance_neighbour(1., vertices, offset, sides + index);
            distance_neighbour(1., vertices, index - 2, sides + offset);
            distance_neighbour(1., vertices, index, sides + offset);
        }
        distance_neighbour(1., vertices, sides - 2, 2 * sides - 1);
        distance_neighbour(1., vertices, sides - 1, 2 * sides - 2);
        distance_neighbour(1., vertices, sides - 1, 2 * sides - 1);
    }

    #[test]
    fn antiprism_ring_edges_even() {
        let sides = 10;
        let shape = make_shape(sides, true);
        let vertices = shape.vertices();
        let sides = sides as usize;

        distance_neighbour(1., vertices, 0, sides + 1);
        distance_neighbour(1., vertices, 1, sides);
        distance_neighbour(1., vertices, 1, sides + 1);
        for index in 2..sides {
            let offset = if index.is_multiple_of(2) {
                index + 1
            } else {
                index - 3
            };
            distance_neighbour(1., vertices, offset, sides + index - 2);
            distance_neighbour(1., vertices, offset, sides + index);
            distance_neighbour(1., vertices, index - 2, sides + offset);
            distance_neighbour(1., vertices, index, sides + offset);
        }
        distance_neighbour(1., vertices, sides - 2, 2 * sides - 2);
        distance_neighbour(1., vertices, sides - 2, 2 * sides - 1);
        distance_neighbour(1., vertices, sides - 1, 2 * sides - 2);
    }
}

