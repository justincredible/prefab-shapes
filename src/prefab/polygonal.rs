use std::ops::AddAssign;

use num_traits::{cast, Float, FloatConst, one, NumCast, Unsigned, zero};

use crate::shapes::Configuration;

pub(super) type PolygonalSides = u16;

/// Represents shapes with arbitrary polygonal faces.
pub(super) trait Polygonal<C, I>
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    /// Number of sides of the polygon.
    fn sides(&self) -> PolygonalSides;

    /// Center angle between two adjacent vertices.
    /// Returns the half and full angle respectively.
    fn angle(&self) -> (C, C) {
        let angle = C::TAU() / cast::<_, C>(self.sides()).unwrap();

        (cast::<_, C>(0.5).unwrap() * angle, angle)
    }

    /// Radius of the polygonal face with unit length edges.
    fn radius(&self) -> C {
        let (half, angle) = self.angle();

        half.cos() / angle.sin()
    }

    /// Cartesian coordinates of the polygon's vertices.
    /// Vertices are in triangle strip order for the polygon,
    /// which may not be correct for implementors.
    fn vertices(&self, left_right: bool) -> Vec<[C; 3]> {
        let mut vertices = Vec::with_capacity(self.sides().into());
        let odd = !self.sides().is_multiple_of(2);
        if odd {
            vertices.push([zero(), self.radius(), zero()]);
        };

        let first = if left_right { -C::one() } else { one() };

        let (half, angle) = self.angle();
        for step in 0..self.sides()/2 {
            let value = if odd {
                angle * cast::<_, C>(step+1).unwrap()
            } else {
                half + angle * cast::<_, C>(step).unwrap()
            };
            let mut point = [
                self.radius() * first * value.sin(),
                self.radius() * value.cos(),
                // Consumers are responsible for centering with respect to depth.
                zero(),
            ];
            vertices.push(point);
            point[0] = -point[0];
            vertices.push(point);
        }

        vertices
    }

    /// Index list of triangles that cover the polygonal face.
    fn indices(&self) -> Vec<I> {
        let mut indices = Vec::with_capacity(3 * (self.sides() as usize - 2));

        let mut a= zero();
        let mut b = I::one();
        let mut c = cast::<_, I>(2).unwrap();
        let inc = b;

        for i in 0..self.sides()-2 {
            indices.push(a);
            indices.push(b);
            indices.push(c);

            if i.is_multiple_of(2) {
                a = c;
            } else {
                b = c;
            }
            c += inc;
        }

        indices
    }

    /// Determines if left-right vertex order is front facing
    fn left_right_front_facing(&self, config: Configuration) -> bool {
        let odd = !self.sides().is_multiple_of(2);

        odd && config.orientation.is_ccw() || !odd && config.orientation.is_cw()
    }
}

#[cfg(test)]
mod tests {
    use crate::{polygon::Polygon, Shape, Shaper};

    use super::super::{
        linear_algebra::magnitude_diff,
        unit_test::{distance_neighbour, epsilon_error},
    };

    fn make_shape(size: u16) -> Shape<f64, u16> {
        Polygon::new(size).shape(Default::default()).expect("Panics occur before this call")
    }

    #[test]
    fn side_length_odd() {
        let shape = make_shape(11);
        let vertices = shape.vertices();

        distance_neighbour(1., vertices, 1, 0);
        for i in 2..vertices.len() {
            distance_neighbour(1., vertices, i, i-2);
        }
        distance_neighbour(1., vertices, vertices.len()-1, vertices.len()-2);
    }

    #[test]
    fn error_total_odd() {
        let shape = make_shape(u16::MAX - 2);
        let vertices = shape.vertices();

        let mut error = 0.;
        error += 1. - magnitude_diff(vertices[1], vertices[0]);
        for i in 2..vertices.len() {
            error += 1. - magnitude_diff(vertices[i], vertices[i-2]);
        }
        error += 1. - magnitude_diff(vertices[vertices.len()-1], vertices[vertices.len()-2]);

        epsilon_error(error);
    }

    #[test]
    fn side_length_even() {
        let shape = make_shape(10);
        let vertices = shape.vertices();

        distance_neighbour(1., vertices, 1, 0);
        for i in 2..vertices.len() {
            distance_neighbour(1., vertices, i, i-2);
        }
        distance_neighbour(1., vertices, vertices.len()-1, vertices.len()-2);
    }

    #[test]
    fn error_total_even() {
        let shape = make_shape(u16::MAX - 9);
        let vertices = shape.vertices();

        let mut error = 0.;
        error += 1. - magnitude_diff(vertices[1], vertices[0]);
        for i in 2..vertices.len() {
            error += 1. - magnitude_diff(vertices[i], vertices[i-2]);
        }
        error += 1. - magnitude_diff(vertices[vertices.len()-1], vertices[vertices.len()-2]);

        epsilon_error(error);
    }
}
