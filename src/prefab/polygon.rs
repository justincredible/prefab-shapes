use std::ops::AddAssign;

use num_traits::{Float, FloatConst, NumCast, one, Unsigned, zero};

use crate::prefab::polygonal::{Polygonal, PolygonalSides};
use crate::shapes::{Configuration, Shape, Shaper, ShapingError};

/// Regular polygons with less than 65536 sides.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Polygon {
    pub(super) sides: u16
}

impl Polygon {
    /// Create a regular polygon with vertices in triangle strip order.
    ///
    /// # Panics
    ///
    /// May panic if `sides` is less than three.
    pub fn new(sides: u16) -> Self {
        if sides < 3 { panic!("degenerate polygon") }

        Self { sides }
    }
}

impl<C, I> Polygonal<C, I> for Polygon
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn sides(&self) -> PolygonalSides {
        self.sides
    }
}

impl<C, I> Shaper<C, I> for Polygon
where
    C: Float + FloatConst,
    I: AddAssign + Copy + NumCast + Unsigned,
{
    fn shape(&self, request: Configuration) -> Result<Shape<C, I>, ShapingError> {
        let left_right = Polygonal::<C, I>::left_right_front_facing(self, request);
        let vertices = Polygonal::<C, I>::vertices(self, left_right);

        if request.prefer_strips {
            Shape::as_strips(vertices, vec![])
        } else {
            let indices = Polygonal::<C, I>::indices(self);

            if request.generate_normals {
                let normals = if request.orientation.is_left() {
                    vec!([zero(), zero(), -C::one()])
                } else {
                    vec!([zero(), zero(), one()])
                };

                Shape::with_normals(vertices, normals, indices)
            } else {
                Shape::without_normals(vertices, indices)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Polygon, Shape, Shaper};

    fn make_shape(size: u16) -> Shape<f64, u16> {
        Polygon::new(size).shape(Default::default()).expect("Panics occur before this call")
    }

    #[test]
    #[should_panic]
    fn zero_sides() {
        make_shape(0);
    }

    #[test]
    #[should_panic]
    fn two_sides() {
        make_shape(2);
    }

    #[test]
    fn three_sides() {
        make_shape(3);
    }

    #[test]
    fn u8_max_sides() {
        let _: Shape<f32, u8> = Polygon::new(255).shape(Default::default()).expect("Panics occur before this call");
    }

    #[test]
    #[should_panic]
    fn u8_overflow() {
        let _: Shape<f32, u8> = Polygon::new(256).shape(Default::default()).expect("Panics occur before this call");
    }

    #[test]
    fn u16_min_sides() {
        let _: Shape<f32, u16> = Polygon::new(256).shape(Default::default()).expect("Panics occur before this call");
    }

    #[test]
    fn max_sides() {
        make_shape(u16::MAX);
    }
}
